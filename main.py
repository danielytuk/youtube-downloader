# YTDL - Windows-only (Python 3.12+)
# PySide6 app with temp toolchain (ffmpeg/ffprobe, yt-dlp, deno),
# WAV-first pipeline, optional Shazam candidate harvesting (vote-based, not scoring),
# aggressive-but-reasonable metadata enrichment (iTunes + song.link fallback),
# and ID3v2.3 tagging (UTF-16 text, v2.3 save) with CD-like frames.
#
# Required pip deps:
#   pip install PySide6 requests mutagen shazamio
#
# Notes:
# - This script downloads binaries to %TEMP%\YTDL_bin at runtime.
# - Enforces single instance via Windows mutex.

from __future__ import annotations

import asyncio
import ctypes
import os
import re
import shutil
import sys
import tempfile
import time
import traceback
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import requests
from PySide6 import QtCore, QtGui, QtWidgets

# Platform guard
if sys.platform != "win32":
    raise SystemExit("This application is Windows-only.")

# Single instance (Windows mutex)
class SingleInstance:
    """
    Enforces one running instance using a named Windows mutex.
    """
    ERROR_ALREADY_EXISTS = 183

    def __init__(self, name: str):
        self.name = name
        self.handle = None

    def acquire(self) -> bool:
        kernel32 = ctypes.windll.kernel32
        kernel32.SetLastError(0)
        self.handle = kernel32.CreateMutexW(None, False, self.name)
        if not self.handle:
            return True
        last = kernel32.GetLastError()
        if last == self.ERROR_ALREADY_EXISTS:
            return False
        return True

    def release(self) -> None:
        if self.handle:
            ctypes.windll.kernel32.CloseHandle(self.handle)
            self.handle = None

# App config
APP_NAME = "YTDL"

USER_AGENT = (
    "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
    "(KHTML, like Gecko) Chrome/130.0.0.0 Safari/537.36"
)

YOUTUBE_RE = re.compile(
    r"(https?://)?(www\.)?"
    r"(youtube\.com/watch\?v=[\w-]{6,}|youtu\.be/[\w-]{6,}|youtube\.com/shorts/[\w-]{6,})",
    re.IGNORECASE,
)

# BIN_DIR in system temp
BIN_DIR = Path(tempfile.gettempdir()) / f"{APP_NAME}_bin"
BIN_DIR.mkdir(parents=True, exist_ok=True)

# Downloads
YTDLP_URL = "https://github.com/yt-dlp/yt-dlp/releases/latest/download/yt-dlp.exe"
YTDLP_PATH = BIN_DIR / "yt-dlp.exe"

# ffmpeg zip contains /bin/ffmpeg.exe + ffprobe.exe
FFMPEG_ZIP_URL = "https://www.gyan.dev/ffmpeg/builds/ffmpeg-release-essentials.zip"
FFMPEG_PATH = BIN_DIR / "ffmpeg.exe"
FFPROBE_PATH = BIN_DIR / "ffprobe.exe"

# Deno (zip contains deno.exe)
DENO_ZIP_URL = "https://github.com/denoland/deno/releases/latest/download/deno-x86_64-pc-windows-msvc.zip"
DENO_PATH = BIN_DIR / "deno.exe"

TOOL_REFRESH_SECONDS = 2 * 60 * 60  # 2 hours

# Requests cooldown (keep for metadata API politeness; NOT used for tool downloads)
REQUEST_COOLDOWN_S = 1.0

# Larger, readable UI (non-resizable)
QSS = """
* { font-family: "Segoe UI", "Inter", "Arial"; font-size: 13px; }
QWidget { background: #0F1115; color: #E7EAF0; }
QDialog { border: 1px solid #232734; border-radius: 14px; }
QLabel#Title { font-size: 18px; font-weight: 700; }
QLabel#Subtle { color: #AAB1C3; font-size: 12px; }
QFrame#Card { background: #121623; border: 1px solid #232734; border-radius: 14px; }
QPushButton {
  background: #171C2B; border: 1px solid #2A3042; border-radius: 10px;
  padding: 10px 14px;
}
QPushButton:hover { background: #1B2236; }
QPushButton:pressed { background: #141A29; }
QPushButton:disabled { color: #717A90; background: #101522; border: 1px solid #1F2433; }
QProgressBar {
  border: 1px solid #2A3042; border-radius: 10px; text-align: center;
  background: #101522; height: 16px;
}
QProgressBar::chunk { background: #3B82F6; border-radius: 10px; }
QLineEdit, QComboBox {
  background: #0F1320; border: 1px solid #2A3042; border-radius: 10px;
  padding: 10px 10px; selection-background-color: #3B82F6;
}
QComboBox::drop-down { border: 0px; width: 26px; }
QCheckBox { spacing: 10px; }
QFormLayout QLabel { font-size: 12px; color: #AAB1C3; }
"""

# Cleanup helpers (YouTube title)
ALBUM_SUFFIX_RE = re.compile(r"\s*[-–—]\s*(single|ep|album)\s*$", re.IGNORECASE)

def clean_album_name(name: str) -> str:
    name = name or ""
    return ALBUM_SUFFIX_RE.sub("", name).strip()

_MARKETING_BAD_WORDS = (
    "official",
    "lyrics",
    "audio",
    "video",
    "music video",
    "visualizer",
    "free download",
    "4k",
    "4k remaster",
    "4k remastered",
)
_MARKETING_BAD_WORDS_SORTED = tuple(sorted(_MARKETING_BAD_WORDS, key=len, reverse=True))
_MARKETING_KEEP_IF_REMIX_WORDS = ("remix", "mix", "edit", "bootleg", "rework", "remaster", "remastered")

_BAD_TOKENS = set()
for phrase in _MARKETING_BAD_WORDS:
    for tok in phrase.lower().split():
        _BAD_TOKENS.add(tok)

_GENERIC_CONNECTORS = {"and", "with", "feat", "featuring", "ft", "vs", "x"}


def _segment_is_marketing(seg: str) -> bool:
    seg_l = seg.lower()
    if any(word in seg_l for word in _MARKETING_KEEP_IF_REMIX_WORDS):
        return False
    tokens = re.findall(r"[a-z0-9']+", seg_l)
    meaningful = [t for t in tokens if t not in _GENERIC_CONNECTORS]
    if not meaningful:
        return False
    return all(t in _BAD_TOKENS for t in meaningful)


def _strip_trailing_marketing(title: str) -> str:
    s = (title or "").strip()
    if not s:
        return s

    while s.endswith((")", "]")):
        close_idx = len(s) - 1
        open_ch = "(" if s.endswith(")") else "["
        open_idx = s.rfind(open_ch)
        if open_idx == -1:
            break

        inside = s[open_idx + 1 : close_idx].strip().lower()
        if any(word in inside for word in _MARKETING_KEEP_IF_REMIX_WORDS):
            break
        if any(word in inside for word in _MARKETING_BAD_WORDS):
            s = s[:open_idx].rstrip()
            continue
        break

    lowered = s.lower()
    for sep in (" - ", " | ", " • "):
        idx = lowered.rfind(sep)
        if idx == -1:
            continue
        right = s[idx + len(sep) :].strip()
        if _segment_is_marketing(right):
            s = s[:idx].rstrip()
            lowered = s.lower()
            break

    while True:
        lowered = s.lower()
        trimmed = False
        for phrase in _MARKETING_BAD_WORDS_SORTED:
            pl = phrase.lower()
            if lowered.endswith(pl):
                idx = len(s) - len(pl)
                if idx == 0 or s[idx - 1] in " []()-•|":
                    s = s[:idx].rstrip()
                    trimmed = True
                    break
        if not trimmed:
            break

    s = re.sub(r"[\-\|\•\s]+$", "", s)
    return s.strip()


def parse_youtube_title(raw_title: str) -> tuple[str, str]:
    if not raw_title:
        return "", ""

    s = _strip_trailing_marketing(raw_title)
    if not s:
        return "", ""

    s = re.sub(r"\s+", " ", s).strip()

    parts = re.split(r"\s[-–—]\s", s, maxsplit=1)
    if len(parts) == 2:
        artist = parts[0].strip(" -–—")
        title_part = parts[1].strip()
    else:
        artist = ""
        title_part = s

    m = re.match(r"^\[(.+)\]$", title_part)
    if m:
        title_part = m.group(1).strip()

    if (title_part.startswith('"') and title_part.endswith('"')) or (
        title_part.startswith("'") and title_part.endswith("'")
    ):
        title_part = title_part[1:-1].strip()

    return artist, title_part.strip()

# Errors + subprocess wrapper
class AppError(RuntimeError):
    pass


def _tail(text: str, max_chars: int = 4000) -> str:
    if not text:
        return ""
    return text if len(text) <= max_chars else text[-max_chars:]


def run_cmd(args: List[str], cwd: Optional[Path] = None, timeout: Optional[int] = None) -> Tuple[str, str]:
    import subprocess

    try:
        p = subprocess.run(
            args,
            cwd=str(cwd) if cwd else None,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            shell=False,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
    except Exception as e:
        raise AppError(f"Failed to start process:\n{args[0]}\n\n{e}") from e

    out = p.stdout or ""
    err = p.stderr or ""
    if p.returncode != 0:
        raise AppError(
            f"Command failed (exit code {p.returncode}).\n\n"
            f"Command:\n  {' '.join(args)}\n\n"
            f"--- stdout (tail) ---\n{_tail(out)}\n\n"
            f"--- stderr (tail) ---\n{_tail(err)}"
        )
    return out, err

# Requests with cooldown (metadata APIs)
class CooldownSession:
    def __init__(self, user_agent: str, cooldown_s: float = 1.0):
        self.s = requests.Session()
        self.s.headers.update({"User-Agent": user_agent})
        self.cooldown_s = max(0.0, float(cooldown_s))
        self._last = 0.0

    def _cooldown(self):
        if self.cooldown_s <= 0:
            return
        now = time.time()
        delta = now - self._last
        if delta < self.cooldown_s:
            time.sleep(self.cooldown_s - delta)
        self._last = time.time()

    def get(self, *args, **kwargs):
        self._cooldown()
        return self.s.get(*args, **kwargs)

    def post(self, *args, **kwargs):
        self._cooldown()
        return self.s.post(*args, **kwargs)

# Toolchain manager (temp BIN_DIR) - NO cooldown for downloads
class Toolchain:
    def __init__(self, user_agent: str):
        self.s = requests.Session()
        self.s.headers.update({"User-Agent": user_agent})

    def ensure_ready(self) -> None:
        BIN_DIR.mkdir(parents=True, exist_ok=True)

        if not (FFMPEG_PATH.exists() and FFPROBE_PATH.exists()):
            self._download_and_extract_ffmpeg()

        self._ensure_fresh_binary(YTDLP_URL, YTDLP_PATH, is_zip=False)
        self._ensure_fresh_binary(DENO_ZIP_URL, DENO_PATH, is_zip=True, zip_member_name="deno.exe")

        os.environ["PATH"] = str(BIN_DIR) + os.pathsep + os.environ.get("PATH", "")

    def _is_stale(self, path: Path, max_age_s: int) -> bool:
        if not path.exists():
            return True
        return (time.time() - path.stat().st_mtime) > max_age_s

    def _download(self, url: str, dest: Path) -> None:
        dest.parent.mkdir(parents=True, exist_ok=True)
        tmp = dest.with_suffix(dest.suffix + ".part")
        with self.s.get(url, stream=True, timeout=90) as r:
            r.raise_for_status()
            with open(tmp, "wb") as f:
                for chunk in r.iter_content(chunk_size=1024 * 256):
                    if chunk:
                        f.write(chunk)
        tmp.replace(dest)

    def _ensure_fresh_binary(self, url: str, dest: Path, is_zip: bool, zip_member_name: Optional[str] = None) -> None:
        if is_zip:
            if not self._is_stale(dest, TOOL_REFRESH_SECONDS):
                return
            zip_path = BIN_DIR / (dest.stem + ".zip")
            self._download(url, zip_path)
            self._extract_zip_member(zip_path, zip_member_name or dest.name, dest)
            try:
                zip_path.unlink(missing_ok=True)
            except Exception:
                pass
        else:
            if not self._is_stale(dest, TOOL_REFRESH_SECONDS):
                return
            self._download(url, dest)

    def _extract_zip_member(self, zip_path: Path, member_name: str, out_path: Path) -> None:
        with zipfile.ZipFile(zip_path, "r") as z:
            candidates = [m for m in z.namelist() if Path(m).name.lower() == member_name.lower()]
            if not candidates:
                raise AppError(f"Zip did not contain {member_name}")
            member = candidates[0]
            tmp = out_path.with_suffix(".part")
            with z.open(member) as src, open(tmp, "wb") as dst:
                shutil.copyfileobj(src, dst)
            tmp.replace(out_path)

    def _download_and_extract_ffmpeg(self) -> None:
        zip_path = BIN_DIR / "ffmpeg-release-essentials.zip"
        self._download(FFMPEG_ZIP_URL, zip_path)

        ffmpeg_member = None
        ffprobe_member = None
        with zipfile.ZipFile(zip_path, "r") as z:
            for name in z.namelist():
                p = Path(name)
                if p.name.lower() == "ffmpeg.exe" and "bin" in p.parts:
                    ffmpeg_member = name
                elif p.name.lower() == "ffprobe.exe" and "bin" in p.parts:
                    ffprobe_member = name

            if not ffmpeg_member or not ffprobe_member:
                raise AppError("Could not locate ffmpeg.exe/ffprobe.exe inside the ffmpeg zip.")

            self._extract_specific_member(z, ffmpeg_member, FFMPEG_PATH)
            self._extract_specific_member(z, ffprobe_member, FFPROBE_PATH)

        try:
            zip_path.unlink(missing_ok=True)
        except Exception:
            pass

    def _extract_specific_member(self, z: zipfile.ZipFile, member: str, out_path: Path) -> None:
        tmp = out_path.with_suffix(".part")
        with z.open(member) as src, open(tmp, "wb") as dst:
            shutil.copyfileobj(src, dst)
        tmp.replace(out_path)

# Metadata structures
@dataclass
class TrackMeta:
    title: str = ""
    artist: str = ""
    album: str = ""
    album_artist: str = ""
    year: str = ""
    genre: str = ""
    track_number: str = ""
    track_total: str = ""
    disc_number: str = ""
    disc_total: str = ""
    isrc: str = ""
    itunes_track_id: str = ""
    artwork_url: str = ""
    artwork_jpeg: bytes = b""
    comment: str = ""  # e.g. Source URL

@dataclass
class CandidateOption:
    label: str
    meta: TrackMeta

# iTunes + song.link helpers
def _norm(s: str) -> str:
    s = (s or "").lower()
    s = re.sub(r"[\(\)\[\]\{\}\-–—_:;,.!/?\\|]+", " ", s)
    s = re.sub(r"\s+", " ", s).strip()
    return s

def _strip_parens_soft(s: str) -> str:
    if not s:
        return ""
    keep_words = ("remix", "edit", "bootleg", "rework", "mix", "cover", "remaster", "remastered")
    if any(w in s.lower() for w in keep_words):
        return s
    return re.sub(r"\s*[\(\[].*?[\)\]]\s*", " ", s).strip()

def _drop_feat_tail(s: str) -> str:
    return re.sub(r"\s*\(?(feat\.|featuring|ft\.)\s+.+?\)?\s*$", "", s, flags=re.IGNORECASE).strip()

def itunes_search_variants(artist: str, title: str) -> List[str]:
    a = (artist or "").strip()
    t = (title or "").strip()
    variants: List[str] = []

    if a and t:
        variants.append(f"{a} {t}")
        variants.append(f"{t} {a}")

    a2 = _strip_parens_soft(a)
    t2 = _strip_parens_soft(t)
    if a2 and t2 and (a2 != a or t2 != t):
        variants.append(f"{a2} {t2}")
        variants.append(f"{t2} {a2}")

    a3 = _drop_feat_tail(a2 or a)
    t3 = _drop_feat_tail(t2 or t)
    if a3 and t3 and (a3 != a or t3 != t):
        variants.append(f"{a3} {t3}")
        variants.append(f"{t3} {a3}")

    out: List[str] = []
    seen = set()
    for v in variants:
        v2 = v.strip()
        if v2 and v2.lower() not in seen:
            seen.add(v2.lower())
            out.append(v2)
    return out[:4]  # keep requests reasonable

def itunes_search(cs: CooldownSession, term: str, limit: int = 12) -> Dict[str, Any]:
    url = "https://itunes.apple.com/search"
    params = {"term": term, "media": "music", "entity": "song", "limit": str(limit)}
    r = cs.get(url, params=params, timeout=25)
    r.raise_for_status()
    return r.json()

def itunes_lookup(cs: CooldownSession, track_id: str) -> Dict[str, Any]:
    url = "https://itunes.apple.com/lookup"
    params = {"id": track_id}
    r = cs.get(url, params=params, timeout=25)
    r.raise_for_status()
    return r.json()

def songlink_lookup(cs: CooldownSession, query_url: str) -> Dict[str, Any]:
    url = "https://api.song.link/v1-alpha.1/links"
    params = {"url": query_url}
    r = cs.get(url, params=params, timeout=25)
    r.raise_for_status()
    return r.json()

def songlink_extract_itunes_id(payload: Dict[str, Any]) -> str:
    if not isinstance(payload, dict):
        return ""
    links = payload.get("linksByPlatform") or {}
    candidate_urls = []
    for key in ("appleMusic", "itunes"):
        v = links.get(key)
        if isinstance(v, dict) and v.get("url"):
            candidate_urls.append(v["url"])

    entities = payload.get("entitiesByUniqueId") or {}
    for ent in entities.values():
        if isinstance(ent, dict) and ent.get("platform") in ("appleMusic", "itunes") and ent.get("url"):
            candidate_urls.append(ent["url"])

    for u in candidate_urls:
        m = re.search(r"/id(\d+)", u)
        if m:
            return m.group(1)
        m = re.search(r"[?&]i=(\d+)", u)
        if m:
            return m.group(1)
    return ""

def pick_best_itunes_result(results: List[Dict[str, Any]], want_artist: str, want_title: str) -> Optional[Dict[str, Any]]:
    wa = _norm(want_artist)
    wt = _norm(want_title)

    best = None
    best_score = -999

    for item in results:
        a = _norm(item.get("artistName", ""))
        t = _norm(item.get("trackName", ""))
        score = 0
        if wt and wt in t:
            score += 6
        if wa and wa in a:
            score += 6
        if wt and t == wt:
            score += 3
        if wa and a == wa:
            score += 3
        if item.get("kind") == "song":
            score += 1

        if "karaoke" in t:
            score -= 2

        if score > best_score:
            best_score = score
            best = item

    return best

def upgrade_artwork_url(artwork_url: str) -> str:
    if not artwork_url:
        return ""
    return re.sub(r"/\d+x\d+bb\.(jpg|png)$", "/1000x1000bb.jpg", artwork_url, flags=re.IGNORECASE)

def download_artwork(cs: CooldownSession, url: str) -> bytes:
    if not url:
        return b""
    r = cs.get(url, timeout=25)
    r.raise_for_status()
    return r.content

def meta_from_itunes_item(item: Dict[str, Any]) -> TrackMeta:
    m = TrackMeta()
    m.title = item.get("trackName", "") or ""
    m.artist = item.get("artistName", "") or ""
    m.album_artist = m.artist
    m.album = item.get("collectionName", "") or ""
    m.genre = item.get("primaryGenreName", "") or ""

    rel = item.get("releaseDate", "")
    if isinstance(rel, str) and len(rel) >= 4 and rel[:4].isdigit():
        m.year = rel[:4]

    tn = item.get("trackNumber")
    tc = item.get("trackCount")
    if tn is not None:
        m.track_number = str(tn)
    if tc is not None:
        m.track_total = str(tc)

    track_id = item.get("trackId")
    if track_id:
        m.itunes_track_id = str(track_id)

    art = item.get("artworkUrl100") or item.get("artworkUrl60") or ""
    m.artwork_url = upgrade_artwork_url(art) if art else ""
    return m

# ffmpeg/ffprobe helpers
def ffprobe_duration_seconds(path: Path) -> float:
    out, _ = run_cmd(
        [
            str(FFPROBE_PATH),
            "-v",
            "error",
            "-show_entries",
            "format=duration",
            "-of",
            "default=noprint_wrappers=1:nokey=1",
            str(path),
        ]
    )
    try:
        return float(out.strip())
    except Exception:
        return 0.0

def wav_slice_normalized(src_wav: Path, dst_wav: Path, start_s: float, dur_s: float = 30.0) -> None:
    run_cmd(
        [
            str(FFMPEG_PATH),
            "-y",
            "-ss",
            f"{start_s:.3f}",
            "-t",
            f"{dur_s:.3f}",
            "-i",
            str(src_wav),
            "-af",
            "loudnorm=I=-16:TP=-1.5:LRA=11",
            "-ac",
            "2",
            "-ar",
            "44100",
            str(dst_wav),
        ]
    )

def wav_to_mp3_high_compat(src_wav: Path, dst_mp3: Path) -> None:
    # 128k CBR, 44.1kHz stereo, ID3v2.3 for maximum compatibility.
    run_cmd(
        [
            str(FFMPEG_PATH),
            "-y",
            "-i",
            str(src_wav),
            "-vn",
            "-ac",
            "2",
            "-ar",
            "44100",
            "-c:a",
            "libmp3lame",
            "-b:a",
            "128k",
            "-write_id3v2",
            "1",
            "-id3v2_version",
            "3",
            str(dst_mp3),
        ]
    )

# Shazam (candidate harvesting; vote-based)
def shazam_recognize_wav(wav_path: Path) -> Dict[str, Any]:
    try:
        from shazamio import Shazam
    except Exception as e:
        raise AppError("Shazam recognition requires 'shazamio'.\n\nInstall it with:\n  pip install shazamio") from e

    loop = asyncio.new_event_loop()
    try:
        asyncio.set_event_loop(loop)
        shazam = Shazam()
        return loop.run_until_complete(shazam.recognize_song(str(wav_path)))
    finally:
        try:
            loop.close()
        except Exception:
            pass

def _shazam_extract_track(payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    if not isinstance(payload, dict):
        return None
    track = payload.get("track")
    if isinstance(track, dict) and ("title" in track or "subtitle" in track):
        return track
    if "title" in payload and "subtitle" in payload:
        return payload
    return None

def _shazam_track_key(track: Dict[str, Any]) -> str:
    title = _norm((track.get("title") or "").strip())
    artist = _norm((track.get("subtitle") or "").strip())
    if not title and not artist:
        return ""
    return f"{title} — {artist}".strip(" —")

def extract_shazam_fields(payload: Dict[str, Any]) -> Tuple[str, str, str]:
    track = payload.get("track") if isinstance(payload, dict) else None
    if not isinstance(track, dict) and isinstance(payload, dict) and "title" in payload:
        track = payload
    if not isinstance(track, dict):
        return "", "", ""

    title = (track.get("title") or "").strip()
    artist = (track.get("subtitle") or "").strip()

    isrc = ""
    sections = track.get("sections")
    if isinstance(sections, list):
        for sec in sections:
            meta = sec.get("metadata") if isinstance(sec, dict) else None
            if isinstance(meta, list):
                for m in meta:
                    if isinstance(m, dict) and (m.get("title") or "").lower() == "isrc":
                        isrc = (m.get("text") or "").strip()
                        break
            if isrc:
                break

    return title, artist, isrc

def shazam_collect_candidates_from_wav(
    src_wav: Path, work_dir: Path, progress_cb
) -> Tuple[List[Dict[str, Any]], Optional[Dict[str, Any]]]:
    """
    Returns: (candidates, best_guess_track)
    best_guess is "most frequent across slices", tie-breaker: has ISRC (useful for lookups).
    """
    duration = ffprobe_duration_seconds(src_wav)
    if duration <= 10:
        raise AppError("Audio is too short to recognize reliably.")

    offsets: List[float] = []
    base = 30.0
    if duration > base + 31:
        offsets.append(base)
    for frac in (0.25, 0.45, 0.65, 0.80):
        t = min(max(0.0, duration * frac), max(0.0, duration - 31.0))
        if not offsets or all(abs(t - o) > 10 for o in offsets):
            offsets.append(t)
    offsets = offsets[:5]

    by_key: Dict[str, Dict[str, Any]] = {}
    votes: Dict[str, int] = {}

    def track_has_isrc(track: Dict[str, Any]) -> bool:
        _, _, isrc = extract_shazam_fields({"track": track})
        return bool(isrc)

    for i, start_s in enumerate(offsets, 1):
        progress_cb(f"Shazam: analyzing slice {i}/{len(offsets)}…")
        slice_wav = work_dir / f"slice_{i}.wav"
        wav_slice_normalized(src_wav, slice_wav, start_s, 30.0)

        try:
            payload = shazam_recognize_wav(slice_wav)
        finally:
            try:
                slice_wav.unlink(missing_ok=True)
            except Exception:
                pass

        track = _shazam_extract_track(payload)
        if not track:
            continue

        key = _shazam_track_key(track)
        if not key:
            continue

        if key not in by_key:
            by_key[key] = track
            votes[key] = 0
        votes[key] += 1

    items = list(by_key.items())
    items.sort(key=lambda kv: (votes.get(kv[0], 0), track_has_isrc(kv[1])), reverse=True)

    candidates = [t for _, t in items]
    best_guess = candidates[0] if candidates else None
    return candidates, best_guess

# Enrichment pipeline
def enrich_from_itunes_and_songlink(
    cs: CooldownSession,
    base: TrackMeta,
    youtube_url: str,
    aggressive: bool = True,
) -> TrackMeta:
    """
    Attempts, in order:
    1) iTunes ISRC search if present
    2) iTunes search variants for artist+title
    3) song.link to obtain iTunes track id -> iTunes lookup
    """
    meta = TrackMeta(**base.__dict__)  # copy

    chosen: Optional[Dict[str, Any]] = None

    # 1) ISRC search
    if meta.isrc:
        try:
            data = itunes_search(cs, f"isrc:{meta.isrc}", limit=10)
            res = data.get("results") or []
            if res:
                chosen = res[0]
        except Exception:
            pass

    # 2) Search variants
    if not chosen and meta.artist and meta.title:
        for term in itunes_search_variants(meta.artist, meta.title) if aggressive else [f"{meta.artist} {meta.title}"]:
            try:
                data = itunes_search(cs, term, limit=15)
                res = data.get("results") or []
                if not res:
                    continue
                picked = pick_best_itunes_result(res, meta.artist, meta.title)
                if picked:
                    chosen = picked
                    break
            except Exception:
                continue

    # 3) song.link fallback (one request + lookup)
    if not chosen:
        try:
            payload = songlink_lookup(cs, youtube_url)
            it_id = songlink_extract_itunes_id(payload)
            if it_id:
                meta.itunes_track_id = it_id
                data = itunes_lookup(cs, it_id)
                res = data.get("results") or []
                if res:
                    chosen = res[0]
        except Exception:
            pass

    # If we got an iTunes track, do lookup for richer fields (best-effort)
    if chosen:
        track_id = chosen.get("trackId")
        if track_id and str(track_id) != (meta.itunes_track_id or ""):
            meta.itunes_track_id = str(track_id)
        try:
            if meta.itunes_track_id:
                data = itunes_lookup(cs, meta.itunes_track_id)
                res = data.get("results") or []
                if res:
                    chosen = res[0]
        except Exception:
            pass

        filled = meta_from_itunes_item(chosen)

        # Merge: prefer iTunes for album/year/genre/artwork; keep ISRC from Shazam if we have it
        if filled.title:
            meta.title = filled.title
        if filled.artist:
            meta.artist = filled.artist
            meta.album_artist = filled.artist
        if filled.album:
            meta.album = filled.album
        if filled.year:
            meta.year = filled.year
        if filled.genre:
            meta.genre = filled.genre
        if filled.track_number:
            meta.track_number = filled.track_number
        if filled.track_total:
            meta.track_total = filled.track_total
        if filled.itunes_track_id:
            meta.itunes_track_id = filled.itunes_track_id
        if filled.artwork_url:
            meta.artwork_url = filled.artwork_url

        if meta.artwork_url and not meta.artwork_jpeg:
            try:
                meta.artwork_jpeg = download_artwork(cs, meta.artwork_url)
            except Exception:
                meta.artwork_jpeg = b""

    meta.album = clean_album_name(meta.album)

    if not meta.album_artist:
        meta.album_artist = meta.artist

    return meta

# ID3v2.3 tagging (CD-like)
def write_id3_tags_v23(mp3_path: Path, meta: TrackMeta) -> None:
    """
    Writes ID3v2.3 tags using widely-recognized frames.
    Uses UTF-16 for v2.3 (UTF-8 is not part of v2.3).
    Saves explicitly as v2.3.
    """
    try:
        from mutagen.id3 import (
            ID3,
            ID3NoHeaderError,
            TIT2,
            TPE1,
            TPE2,
            TALB,
            TRCK,
            TPOS,
            TCON,
            TYER,
            TSRC,
            COMM,
            APIC,
            TXXX,
        )
    except Exception as e:
        raise AppError("Tag writing requires 'mutagen'.\n\nInstall it with:\n  pip install mutagen") from e

    ENC = 1  # UTF-16 for ID3v2.3 compatibility

    try:
        tags = ID3(str(mp3_path))
    except ID3NoHeaderError:
        tags = ID3()

    def set_one(frame):
        tags.delall(frame.FrameID)
        tags.add(frame)

    def set_txxx(desc: str, value: str):
        tags.delall(f"TXXX:{desc}")
        if value:
            tags.add(TXXX(encoding=ENC, desc=desc, text=value))

    if meta.title:
        set_one(TIT2(encoding=ENC, text=meta.title))
    if meta.artist:
        set_one(TPE1(encoding=ENC, text=meta.artist))
    if meta.album_artist:
        set_one(TPE2(encoding=ENC, text=meta.album_artist))
    if meta.album:
        set_one(TALB(encoding=ENC, text=meta.album))

    if meta.track_number:
        tr = meta.track_number.strip()
        if meta.track_total and "/" not in tr:
            tr = f"{tr}/{meta.track_total.strip()}"
        set_one(TRCK(encoding=ENC, text=tr))

    if meta.disc_number:
        ds = meta.disc_number.strip()
        if meta.disc_total and "/" not in ds:
            ds = f"{ds}/{meta.disc_total.strip()}"
        set_one(TPOS(encoding=ENC, text=ds))

    if meta.genre:
        set_one(TCON(encoding=ENC, text=meta.genre))

    if meta.year:
        y = meta.year.strip()
        m = re.match(r"^(\d{4})", y)
        if m:
            y = m.group(1)
        set_one(TYER(encoding=ENC, text=y))

    if meta.isrc:
        tags.delall("TSRC")
        tags.add(TSRC(encoding=ENC, text=meta.isrc))

    if meta.comment:
        tags.delall("COMM")
        tags.add(COMM(encoding=ENC, lang="eng", desc="Source", text=meta.comment))

    set_txxx("ARTISTS", meta.artist or "")
    set_txxx("ALBUMARTIST", meta.album_artist or "")
    if meta.isrc:
        set_txxx("ISRC", meta.isrc)
        set_txxx("TSRC", meta.isrc)

    if meta.artwork_jpeg:
        tags.delall("APIC")
        tags.add(
            APIC(
                encoding=ENC,
                mime="image/jpeg",
                type=3,
                desc="Cover",
                data=meta.artwork_jpeg,
            )
        )

    tags.save(str(mp3_path), v2_version=3)

def copy_with_collision(src: Path, dest_dir: Path, base_name: str) -> Path:
    dest_dir.mkdir(parents=True, exist_ok=True)
    safe = re.sub(r'[<>:"/\\|?*]+', "_", base_name).strip() or "audio"
    dest = dest_dir / f"{safe}.mp3"
    if not dest.exists():
        shutil.copy2(src, dest)
        return dest

    i = 2
    while True:
        cand = dest_dir / f"{safe} ({i}).mp3"
        if not cand.exists():
            shutil.copy2(src, cand)
            return cand
        i += 1

# Progress plan (deterministic)
@dataclass
class ProgressPlan:
    tools: int = 5
    yt_title: int = 12
    download: int = 45
    to_wav: int = 58
    shazam: int = 72
    enrich: int = 88
    to_mp3: int = 96
    done: int = 100

# Worker: full pipeline (WAV-first)
class PipelineWorker(QtCore.QObject):
    progress_text = QtCore.Signal(str)
    progress_value = QtCore.Signal(int)
    error = QtCore.Signal(str)
    finished = QtCore.Signal(object, object)  # (mp3_path: Path, options: List[CandidateOption])

    def __init__(self, url: str, want_shazam: bool, run_dir: Path):
        super().__init__()
        self.url = url
        self.want_shazam = want_shazam
        self.run_dir = run_dir

        self.cs = CooldownSession(USER_AGENT, cooldown_s=REQUEST_COOLDOWN_S)
        self.plan = ProgressPlan()

    @QtCore.Slot()
    def run(self):
        try:
            mp3_path, options = self._run_impl()
            self.finished.emit(mp3_path, options)
        except AppError as e:
            self.error.emit(str(e))
        except Exception:
            self.error.emit("An unexpected error occurred.\n\n" + traceback.format_exc())

    def _set(self, pct: int, text: str):
        self.progress_value.emit(int(max(0, min(100, pct))))
        self.progress_text.emit(text)

    def _run_impl(self) -> Tuple[Path, List[CandidateOption]]:
        self._set(self.plan.tools, "Preparing tools…")
        Toolchain(USER_AGENT).ensure_ready()

        dl_dir = self.run_dir / "dl"
        out_dir = self.run_dir / "out"
        dl_dir.mkdir(parents=True, exist_ok=True)
        out_dir.mkdir(parents=True, exist_ok=True)

        self._set(self.plan.yt_title, "Fetching YouTube title…")
        yt_title = self._ytdlp_get_title(self.url)

        self._set(self.plan.download, "Downloading best audio…")
        downloaded = self._ytdlp_download_audio(self.url, dl_dir)

        self._set(self.plan.to_wav, "Converting to WAV (analysis)…")
        wav_path = out_dir / "audio.wav"
        self._ffmpeg_to_wav(downloaded, wav_path)

        # Base meta from YouTube title
        fa, ft = parse_youtube_title(yt_title)
        base = TrackMeta(
            title=ft or yt_title or "",
            artist=fa or "",
            album_artist=fa or "",
            comment=self.url,
        )

        candidates: List[Dict[str, Any]] = []
        best_track: Optional[Dict[str, Any]] = None

        if self.want_shazam:
            self._set(self.plan.shazam, "Shazam: finding matches…")
            candidates, best_track = shazam_collect_candidates_from_wav(wav_path, self.run_dir, self.progress_text.emit)

        options: List[CandidateOption] = []

        self._set(self.plan.enrich, "Enriching metadata…")

        base_enriched = enrich_from_itunes_and_songlink(self.cs, base, self.url, aggressive=True)
        options.append(CandidateOption(label="YouTube title (fallback)", meta=base_enriched))

        if candidates:
            ordered = []
            if best_track is not None:
                ordered.append(best_track)
                for t in candidates:
                    if _shazam_track_key(t) != _shazam_track_key(best_track):
                        ordered.append(t)
            else:
                ordered = candidates[:]

            ordered = ordered[:4]

            for track in ordered:
                stitle, sartist, sisrc = extract_shazam_fields({"track": track})

                cmeta = TrackMeta(
                    title=stitle or base.title,
                    artist=sartist or base.artist,
                    album_artist=sartist or base.album_artist,
                    isrc=sisrc or "",
                    comment=self.url,
                )

                enriched = enrich_from_itunes_and_songlink(self.cs, cmeta, self.url, aggressive=True)

                if not enriched.artwork_jpeg and base_enriched.artwork_jpeg:
                    enriched.artwork_jpeg = base_enriched.artwork_jpeg
                    enriched.artwork_url = base_enriched.artwork_url

                label = f"Shazam: {enriched.artist} — {enriched.title}".strip(" —")
                if enriched.isrc:
                    label += " (ISRC)"
                options.append(CandidateOption(label=label, meta=enriched))

        self._set(self.plan.to_mp3, "Converting to high-compat MP3…")
        mp3_path = out_dir / "final.mp3"
        wav_to_mp3_high_compat(wav_path, mp3_path)

        self._set(self.plan.done, "Done.")
        return mp3_path, options

    def _ytdlp_get_title(self, url: str) -> str:
        out, _ = run_cmd(
            [
                str(YTDLP_PATH),
                "--skip-download",
                "--print",
                "%(title)s",
                "--no-warnings",
                url,
            ],
            cwd=self.run_dir,
        )
        return (out or "").strip()

    def _ytdlp_download_audio(self, url: str, dl_dir: Path) -> Path:
        tmpl = str(dl_dir / "%(id)s.%(ext)s")
        args = [
            str(YTDLP_PATH),
            url,
            "-f",
            "bestaudio/best",
            "--no-warnings",
            "--no-playlist",
            "--retries",
            "10",
            "--fragment-retries",
            "10",
            "--concurrent-fragments",
            "8",
            "--extractor-retries",
            "5",
            "--socket-timeout",
            "20",
            "--http-chunk-size",
            "10M",
            "--newline",
            "-o",
            tmpl,
            "--js-runtime",
            str(DENO_PATH),
        ]
        run_cmd(args, cwd=self.run_dir)

        files = list(dl_dir.glob("*.*"))
        if not files:
            raise AppError("yt-dlp finished but no audio file was found.")
        files.sort(key=lambda p: p.stat().st_mtime, reverse=True)
        return files[0]

    def _ffmpeg_to_wav(self, src: Path, dst: Path) -> None:
        run_cmd(
            [
                str(FFMPEG_PATH),
                "-y",
                "-i",
                str(src),
                "-vn",
                "-ac",
                "2",
                "-ar",
                "44100",
                "-c:a",
                "pcm_s16le",
                str(dst),
            ],
            cwd=self.run_dir,
        )

# UI: Busy dialog (non-resizable)
class BusyDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle(APP_NAME)
        self.setModal(True)

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(16, 16, 16, 16)
        lay.setSpacing(12)

        title = QtWidgets.QLabel("Working…")
        title.setObjectName("Title")

        self.line1 = QtWidgets.QLabel("Starting…")
        self.line1.setObjectName("Subtle")

        self.line2 = QtWidgets.QLabel("")
        self.line2.setObjectName("Subtle")
        self.line2.setWordWrap(True)

        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)

        card = QtWidgets.QFrame()
        card.setObjectName("Card")
        card_l = QtWidgets.QVBoxLayout(card)
        card_l.setContentsMargins(16, 16, 16, 16)
        card_l.setSpacing(10)
        card_l.addWidget(title)
        card_l.addWidget(self.line1)
        card_l.addWidget(self.line2)
        card_l.addSpacing(6)
        card_l.addWidget(self.progress)

        lay.addWidget(card)

        self._lock_size()

    def _lock_size(self):
        self.adjustSize()
        s = self.sizeHint()
        self.setFixedSize(max(560, s.width()), max(260, s.height()))

    def set_status(self, text: str):
        self.line2.setText(text)

    def set_progress(self, value: int):
        self.progress.setValue(int(value))

# UI: Metadata dialog (dropdown options; non-resizable)
class MetaDialog(QtWidgets.QDialog):
    saved = QtCore.Signal()

    def __init__(self, mp3_path: Path, options: List[CandidateOption], parent=None):
        super().__init__(parent)
        self.setWindowTitle(APP_NAME)
        self.setModal(True)

        self.mp3_path = mp3_path
        self.options = options or [CandidateOption("Fallback", TrackMeta())]
        self.current_meta = self.options[0].meta

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)

        header = QtWidgets.QLabel("Review metadata")
        header.setObjectName("Title")
        sub = QtWidgets.QLabel("Pick the closest match (remixes/covers included), edit if needed, then save.")
        sub.setObjectName("Subtle")
        sub.setWordWrap(True)

        sel_row = QtWidgets.QHBoxLayout()
        sel_label = QtWidgets.QLabel("Detected matches")
        sel_label.setObjectName("Subtle")

        self.combo = QtWidgets.QComboBox()
        for opt in self.options:
            self.combo.addItem(opt.label)
        self.combo.currentIndexChanged.connect(self._on_choice)

        sel_row.addWidget(sel_label)
        sel_row.addSpacing(10)
        sel_row.addWidget(self.combo, 1)

        top = QtWidgets.QHBoxLayout()
        top.setSpacing(12)

        self.cover = QtWidgets.QLabel()
        self.cover.setFixedSize(160, 160)
        self.cover.setScaledContents(True)
        self.cover.setStyleSheet("border: 1px solid #232734; border-radius: 10px; background: #101522;")

        form = QtWidgets.QFormLayout()
        form.setLabelAlignment(QtCore.Qt.AlignLeft)
        form.setFormAlignment(QtCore.Qt.AlignTop)
        form.setHorizontalSpacing(14)
        form.setVerticalSpacing(10)

        self.e_title = QtWidgets.QLineEdit()
        self.e_artist = QtWidgets.QLineEdit()
        self.e_album = QtWidgets.QLineEdit()
        self.e_album_artist = QtWidgets.QLineEdit()
        self.e_year = QtWidgets.QLineEdit()
        self.e_genre = QtWidgets.QLineEdit()
        self.e_track = QtWidgets.QLineEdit()
        self.e_total = QtWidgets.QLineEdit()
        self.e_disc = QtWidgets.QLineEdit()
        self.e_disc_total = QtWidgets.QLineEdit()
        self.e_isrc = QtWidgets.QLineEdit()

        self.e_year.setPlaceholderText("YYYY")

        form.addRow("Title", self.e_title)
        form.addRow("Artist", self.e_artist)
        form.addRow("Album", self.e_album)
        form.addRow("Album Artist", self.e_album_artist)
        form.addRow("Year", self.e_year)
        form.addRow("Genre", self.e_genre)

        track_row = QtWidgets.QHBoxLayout()
        track_row.addWidget(self.e_track)
        track_row.addWidget(QtWidgets.QLabel("/"))
        track_row.addWidget(self.e_total)
        track_w = QtWidgets.QWidget()
        track_w.setLayout(track_row)
        form.addRow("Track #", track_w)

        disc_row = QtWidgets.QHBoxLayout()
        disc_row.addWidget(self.e_disc)
        disc_row.addWidget(QtWidgets.QLabel("/"))
        disc_row.addWidget(self.e_disc_total)
        disc_w = QtWidgets.QWidget()
        disc_w.setLayout(disc_row)
        form.addRow("Disc #", disc_w)

        form.addRow("ISRC", self.e_isrc)

        form_wrap = QtWidgets.QWidget()
        form_wrap.setLayout(form)

        top.addWidget(self.cover)
        top.addWidget(form_wrap, 1)

        btns = QtWidgets.QHBoxLayout()
        btns.addStretch(1)
        self.save_btn = QtWidgets.QPushButton("Save to Downloads")
        self.close_btn = QtWidgets.QPushButton("Close")
        self.save_btn.clicked.connect(self._save)
        self.close_btn.clicked.connect(self.reject)
        btns.addWidget(self.close_btn)
        btns.addWidget(self.save_btn)

        card = QtWidgets.QFrame()
        card.setObjectName("Card")
        card_l = QtWidgets.QVBoxLayout(card)
        card_l.setContentsMargins(16, 16, 16, 16)
        card_l.setSpacing(12)
        card_l.addWidget(header)
        card_l.addWidget(sub)
        card_l.addLayout(sel_row)
        card_l.addLayout(top)

        root.addWidget(card)
        root.addLayout(btns)

        self._apply_meta(self.current_meta)
        self._lock_size()

    def _lock_size(self):
        self.adjustSize()
        s = self.sizeHint()
        self.setFixedSize(max(860, s.width()), max(520, s.height()))

    def _set_cover(self, jpeg: bytes):
        if not jpeg:
            self.cover.setPixmap(QtGui.QPixmap())
            self.cover.setText("No cover")
            self.cover.setAlignment(QtCore.Qt.AlignCenter)
            return
        img = QtGui.QImage.fromData(jpeg)
        if img.isNull():
            self.cover.setPixmap(QtGui.QPixmap())
            self.cover.setText("No cover")
            self.cover.setAlignment(QtCore.Qt.AlignCenter)
            return
        self.cover.setPixmap(QtGui.QPixmap.fromImage(img))

    def _apply_meta(self, meta: TrackMeta):
        self.current_meta = meta
        self.e_title.setText(meta.title or "")
        self.e_artist.setText(meta.artist or "")
        self.e_album.setText(meta.album or "")
        self.e_album_artist.setText(meta.album_artist or "")
        self.e_year.setText(meta.year or "")
        self.e_genre.setText(meta.genre or "")
        self.e_track.setText(meta.track_number or "")
        self.e_total.setText(meta.track_total or "")
        self.e_disc.setText(meta.disc_number or "")
        self.e_disc_total.setText(meta.disc_total or "")
        self.e_isrc.setText(meta.isrc or "")
        self._set_cover(meta.artwork_jpeg)

    def _on_choice(self, idx: int):
        if idx < 0 or idx >= len(self.options):
            return
        self._apply_meta(self.options[idx].meta)

    def _save(self):
        meta = TrackMeta(**self.current_meta.__dict__)

        meta.title = self.e_title.text().strip()
        meta.artist = self.e_artist.text().strip()
        meta.album = self.e_album.text().strip()
        meta.album_artist = self.e_album_artist.text().strip() or meta.artist
        meta.year = self.e_year.text().strip()
        meta.genre = self.e_genre.text().strip()
        meta.track_number = self.e_track.text().strip()
        meta.track_total = self.e_total.text().strip()
        meta.disc_number = self.e_disc.text().strip()
        meta.disc_total = self.e_disc_total.text().strip()
        meta.isrc = self.e_isrc.text().strip()

        downloads = Path(os.environ.get("USERPROFILE", str(Path.home()))) / "Downloads"
        base_name = f"{meta.artist} - {meta.title}".strip(" -") or "audio"

        try:
            out_path = copy_with_collision(self.mp3_path, downloads, base_name)
            write_id3_tags_v23(out_path, meta)
        except AppError as e:
            QtWidgets.QMessageBox.critical(self, APP_NAME, str(e))
            return
        except Exception:
            QtWidgets.QMessageBox.critical(self, APP_NAME, "Failed to save.\n\n" + traceback.format_exc())
            return

        QtWidgets.QMessageBox.information(self, APP_NAME, f"Saved:\n{out_path}")
        self.saved.emit()
        self.accept()

# UI: Main window (fixed size) + QoL
class MainWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(APP_NAME)
        self.setAcceptDrops(True)

        self.settings = QtCore.QSettings(APP_NAME, APP_NAME)

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(16, 16, 16, 16)
        lay.setSpacing(12)

        title = QtWidgets.QLabel("YTDL")
        title.setObjectName("Title")

        sub = QtWidgets.QLabel(
            "Paste a YouTube link. The app downloads audio, tries to detect metadata (including remixes/covers), "
            "and saves a high-compat MP3 with ID3v2.3 tags."
        )
        sub.setObjectName("Subtle")
        sub.setWordWrap(True)

        self.edit = QtWidgets.QLineEdit()
        self.edit.setPlaceholderText("https://www.youtube.com/watch?v=…")
        self.edit.returnPressed.connect(self.on_go)

        # Changed per request:
        self.try_shazam_cb = QtWidgets.QCheckBox("Try Shazam? (slower, more matches)")
        self.try_shazam_cb.setChecked(self.settings.value("try_shazam", True, bool))

        # Buttons row (QoL)
        btn_row = QtWidgets.QHBoxLayout()
        self.paste_btn = QtWidgets.QPushButton("Paste")
        self.clear_btn = QtWidgets.QPushButton("Clear")
        self.go_btn = QtWidgets.QPushButton("Go")
        self.paste_btn.clicked.connect(self._paste)
        self.clear_btn.clicked.connect(self._clear)
        self.go_btn.clicked.connect(self.on_go)

        btn_row.addWidget(self.paste_btn)
        btn_row.addWidget(self.clear_btn)
        btn_row.addStretch(1)
        btn_row.addWidget(self.go_btn)

        self.hint = QtWidgets.QLabel("")
        self.hint.setObjectName("Subtle")
        self.hint.setWordWrap(True)

        card = QtWidgets.QFrame()
        card.setObjectName("Card")
        card_l = QtWidgets.QVBoxLayout(card)
        card_l.setContentsMargins(16, 16, 16, 16)
        card_l.setSpacing(12)
        card_l.addWidget(title)
        card_l.addWidget(sub)
        card_l.addSpacing(4)
        card_l.addWidget(self.edit)
        card_l.addWidget(self.try_shazam_cb)
        card_l.addLayout(btn_row)
        card_l.addWidget(self.hint)

        lay.addWidget(card)

        self._busy: Optional[BusyDialog] = None
        self._thread: Optional[QtCore.QThread] = None
        self._worker: Optional[PipelineWorker] = None
        self._run_dir: Optional[Path] = None

        self.edit.textChanged.connect(self._on_text)
        self.try_shazam_cb.toggled.connect(self._on_shazam_toggled)

        self._set_running(False)
        self._on_text(self.edit.text())
        self._lock_size()

    def _lock_size(self):
        self.adjustSize()
        s = self.sizeHint()
        self.setFixedSize(max(720, s.width()), max(380, s.height()))

    def _on_shazam_toggled(self, v: bool):
        self.settings.setValue("try_shazam", bool(v))

    def _paste(self):
        text = QtWidgets.QApplication.clipboard().text().strip()
        if text:
            self.edit.setText(text)

    def _clear(self):
        self.edit.setText("")

    def _valid_url(self, text: str) -> bool:
        return bool(YOUTUBE_RE.search(text or ""))

    def _set_running(self, running: bool):
        self.edit.setEnabled(not running)
        self.try_shazam_cb.setEnabled(not running)
        self.paste_btn.setEnabled(not running)
        self.clear_btn.setEnabled(not running)
        # go enabled only if valid URL and not running
        self.go_btn.setEnabled((not running) and self._valid_url(self.edit.text()))

    def _on_text(self, text: str):
        if self._valid_url(text):
            self.hint.setText("YouTube link detected. Press Go (or Enter).")
        else:
            self.hint.setText("")
        self.go_btn.setEnabled(self._valid_url(text) and self.edit.isEnabled())

    # Drag & drop QoL
    def dragEnterEvent(self, event: QtGui.QDragEnterEvent):
        if event.mimeData().hasText():
            event.acceptProposedAction()

    def dropEvent(self, event: QtGui.QDropEvent):
        text = (event.mimeData().text() or "").strip()
        if text:
            self.edit.setText(text)

    def on_go(self):
        url = self.edit.text().strip()
        if not self._valid_url(url):
            QtWidgets.QMessageBox.warning(self, APP_NAME, "Please enter a valid YouTube URL.")
            return

        # Per request: no "ask" screen. Checkbox is direct.
        want_shazam = bool(self.try_shazam_cb.isChecked())

        self._start_pipeline(url, want_shazam)

    def _start_pipeline(self, url: str, want_shazam: bool):
        self._set_running(True)
        self._run_dir = Path(tempfile.mkdtemp(prefix=f"{APP_NAME}_run_"))

        if self._busy is None:
            self._busy = BusyDialog(self)

        self._busy.set_status("Starting…")
        self._busy.set_progress(0)
        self._busy.show()
        self._busy.raise_()
        self._busy.activateWindow()

        self._thread = QtCore.QThread(self)
        self._worker = PipelineWorker(url=url, want_shazam=want_shazam, run_dir=self._run_dir)
        self._worker.moveToThread(self._thread)

        self._thread.started.connect(self._worker.run)
        self._worker.progress_text.connect(self._busy.set_status)
        self._worker.progress_value.connect(self._busy.set_progress)
        self._worker.error.connect(self._on_error)
        self._worker.finished.connect(self._on_finished)

        self._thread.start()

    def _cleanup_thread(self):
        if self._thread:
            self._thread.quit()
            self._thread.wait(8000)
            self._thread.deleteLater()
        self._thread = None
        if self._worker:
            self._worker.deleteLater()
        self._worker = None

    def _on_error(self, msg: str):
        self._cleanup_thread()
        if self._busy:
            self._busy.hide()
        QtWidgets.QMessageBox.critical(self, APP_NAME, msg)

        self._cleanup_run_dir()
        self.edit.setText("")
        self._set_running(False)

    def _on_finished(self, mp3_path: Path, options: List[CandidateOption]):
        self._cleanup_thread()
        if self._busy:
            self._busy.hide()

        dlg = MetaDialog(mp3_path=mp3_path, options=options, parent=self)

        def on_saved():
            self.edit.setText("")
            self._cleanup_run_dir()

        dlg.saved.connect(on_saved)

        result = dlg.exec()

        # Per request: if user closes Review Metadata, clear URL input.
        # Also cleanup temp output if not saved.
        if result != QtWidgets.QDialog.Accepted:
            self.edit.setText("")
            self._cleanup_run_dir()

        self._set_running(False)

    def _cleanup_run_dir(self):
        if self._run_dir and self._run_dir.exists():
            try:
                shutil.rmtree(self._run_dir, ignore_errors=True)
            except Exception:
                pass
        self._run_dir = None

# PyInstaller resource path + icon
def resource_path(relative: str) -> str:
    base_path = getattr(sys, "_MEIPASS", None)
    if base_path:
        return str(Path(base_path) / relative)
    return str(Path(__file__).resolve().parent / relative)

def set_app_icon(app: QtWidgets.QApplication):
    try:
        ico_path = Path(resource_path("icon.ico"))
        if ico_path.exists():
            app.setWindowIcon(QtGui.QIcon(str(ico_path)))
    except Exception:
        pass

# Main
def main():
    mutex = SingleInstance(r"Global\YTDL_SINGLE_INSTANCE_MUTEX_v1")
    if not mutex.acquire():
        app = QtWidgets.QApplication(sys.argv)
        app.setStyleSheet(QSS)
        QtWidgets.QMessageBox.information(None, APP_NAME, "YTDL is already running.")
        return

    try:
        QtCore.QCoreApplication.setApplicationName(APP_NAME)
        app = QtWidgets.QApplication(sys.argv)
        app.setStyleSheet(QSS)
        set_app_icon(app)

        w = MainWidget()
        w.show()

        app.exec()
    finally:
        mutex.release()

if __name__ == "__main__":
    main()

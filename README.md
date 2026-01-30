# YTDL App (Windows) 🎧

A tiny, **PySide6** app for turning a YouTube link into a **high-compatibility MP3** with best-effort **metadata + cover art**, and optional **Shazam-based match suggestions**.

> **Windows-only. Python 3.12+ required.**  
> The app downloads `yt-dlp`, `ffmpeg/ffprobe`, and `deno` into your temp folder at runtime.

---

## What it does

- Paste a YouTube URL → downloads **best audio** via `yt-dlp`
- Converts to **WAV-first** for analysis, then outputs **MP3 (128k CBR, 44.1kHz stereo)**
- Optional: runs **Shazam candidate harvesting** (vote-based across slices)
- Enriches metadata using:
  - **iTunes Search/Lookup**
  - **song.link** fallback → iTunes ID → lookup
- Writes **ID3v2.3 tags** (UTF-16) for maximum player compatibility
- Saves the final MP3 to your **Downloads** folder
- Enforces **single instance** via a Windows mutex

---

## Requirements

* **Windows**
* **Python 3.12+**
* Pip dependencies:

```bash
pip install PySide6 requests mutagen shazamio
```

> `shazamio` is only needed if you want the “Try Shazam?” option.

---

## Run

```bash
python ytdl.py
```

1. Paste a YouTube link
2. (Optional) enable **Try Shazam?**
3. Click **Go**
4. Pick the best metadata match, edit if needed, then **Save to Downloads**

---

## How the temp toolchain works

On first run (and then periodically), YTDL downloads tools into:

* `%TEMP%\YTDL_bin`

Tools pulled at runtime:

* `yt-dlp.exe` (latest)
* `ffmpeg.exe` + `ffprobe.exe` (gyan.dev essentials build)
* `deno.exe` (latest)

They’re refreshed roughly every **2 hours** (based on file modified time).

---

## Notes & behavior

* **Non-resizable UI** (designed for a clean fixed layout)
* Drag & drop a URL into the window
* If you close the metadata review dialog without saving, temp output is cleaned up
* Output tagging targets broad compatibility:

  * **ID3v2.3**
  * UTF-16 text frames
  * common “CD-like” frames + embedded cover art (APIC)

---

## Disclaimer

This tool is intended for personal/legitimate use and is provided as-is.

Please respect the rights of creators and platforms you download from.

ChatGPT was used for file clean-up, fixes and this README.

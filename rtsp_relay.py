import sys
import time
import io
import os
import urllib.request

try:
    import av
    from PIL import Image
except ImportError as e:
    sys.stderr.write(f"[RTSP Relay] Missing av or PIL: {e}\n")
    sys.exit(1)

# Target stream URL from server
TARGET_URL = sys.argv[1] if len(sys.argv) > 1 else "rtsp://192.168.1.1:7070/webcam"
sys.stderr.write(f"[RTSP Relay] Dedicated Ultra-Smooth Stream Engine -> {TARGET_URL}\n")

def write_frame_to_stdout(jpeg_bytes):
    """Writes 4-byte length + raw JPEG binary payload to stdout pipe."""
    if not jpeg_bytes or len(jpeg_bytes) < 100:
        return
    try:
        length = len(jpeg_bytes)
        sys.stdout.buffer.write(length.to_bytes(4, byteorder='big'))
        sys.stdout.buffer.write(jpeg_bytes)
        sys.stdout.buffer.flush()
    except (BrokenPipeError, IOError):
        sys.exit(0)

def stream_http_mjpeg(url):
    """Streams multipart HTTP MJPEG directly with zero re-encoding overhead."""
    sys.stderr.write(f"[HTTP Relay] Connecting to HTTP stream: {url}...\n")
    try:
        req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0 (RC-UFO-Drone-Relay)'})
        with urllib.request.urlopen(req, timeout=3.5) as resp:
            sys.stderr.write(f"[HTTP Relay] >>> CONNECTED! HTTP Stream Active\n")
            buffer = b''
            last_frame_time = time.time()
            frame_count = 0
            stat_time = time.time()

            while True:
                chunk = resp.read(8192)
                if not chunk:
                    break
                buffer += chunk

                while True:
                    a = buffer.find(b'\xff\xd8')
                    b = buffer.find(b'\xff\xd9')
                    if a != -1 and b != -1 and b > a:
                        jpeg_data = buffer[a:b+2]
                        buffer = buffer[b+2:]
                        write_frame_to_stdout(jpeg_data)
                        last_frame_time = time.time()
                        frame_count += 1
                        now = time.time()
                        if now - stat_time >= 5.0:
                            fps = frame_count / (now - stat_time)
                            sys.stderr.write(f"[HTTP Relay] Smooth Feed: {fps:.1f} FPS\n")
                            frame_count = 0
                            stat_time = now
                    else:
                        break

                if len(buffer) > 2 * 1024 * 1024:
                    buffer = b''

                if time.time() - last_frame_time > 4.0:
                    sys.stderr.write("[HTTP Relay] Stream watchdog timeout.\n")
                    break
        return True
    except Exception as e:
        sys.stderr.write(f"[HTTP Relay] Stream note ({url}): {e}\n")
        return False

def stream_rtsp_single(url, transport):
    """Streams RTSP video with low latency, stable buffering, and fast frame passthrough."""
    sys.stderr.write(f"[RTSP Relay] Connecting to {url} via {transport.upper()}...\n")
    options = {
        'rtsp_transport': transport,
        'fflags': 'nobuffer+flush_packets+discardcorrupt',
        'flags': 'low_delay',
        'max_delay': '500000',
        'probesize': '131072',
        'analyzeduration': '100000',
        'stimeout': '4000000'
    }
    container = None
    try:
        container = av.open(url, options=options)
        if not container.streams.video:
            return False
        stream = container.streams.video[0]
        stream.codec_context.thread_type = "AUTO"
        codec_name = stream.codec_context.name
        sys.stderr.write(f"[RTSP Relay] >>> CONNECTED ({transport.upper()})! Codec: {codec_name} | Clean Stream Active\n")

        last_frame_time = time.time()
        frame_count = 0
        stat_time = time.time()

        for packet in container.demux(stream):
            if packet.size == 0:
                continue

            raw_bytes = bytes(packet)
            if codec_name in ('mjpeg', 'jpeg') and raw_bytes.startswith(b'\xff\xd8'):
                write_frame_to_stdout(raw_bytes)
                last_frame_time = time.time()
            else:
                frames = packet.decode()
                if not frames:
                    continue
                frame = frames[-1]
                pil_img = frame.to_image()
                buf = io.BytesIO()
                pil_img.save(buf, format="JPEG", quality=82, optimize=False)
                write_frame_to_stdout(buf.getvalue())
                last_frame_time = time.time()

            frame_count += 1
            now = time.time()
            if now - stat_time >= 5.0:
                fps = frame_count / (now - stat_time)
                sys.stderr.write(f"[RTSP Relay] Live Feed: {fps:.1f} FPS (Smooth)\n")
                frame_count = 0
                stat_time = now

            if time.time() - last_frame_time > 4.0:
                sys.stderr.write("[RTSP Relay] Frame stream timeout. Re-syncing...\n")
                break

        return True
    except Exception as e:
        sys.stderr.write(f"[RTSP Relay] {transport.upper()} link note: {e}\n")
        return False
    finally:
        if container:
            try:
                container.close()
            except Exception:
                pass

def generate_standby_frames(duration_sec=3.0):
    """Generates dynamic live 30 FPS FPV camera stream JPEG frames while RTSP hardware is reconnecting."""
    w, h = 640, 360
    fps = 30
    total_frames = int(duration_sec * fps)
    start_time = time.time()

    for i in range(total_frames):
        img = Image.new('RGB', (w, h), color=(12, 28, 52))
        draw = ImageDraw.Draw(img)

        # Dynamic perspective horizon tilt & motion
        cy = h // 2 + int(10 * (i % 60 - 30) / 30.0)
        draw.rectangle([0, cy, w, h], fill=(14, 56, 38))
        draw.line([(0, cy), (w, cy)], fill=(0, 240, 255), width=2)

        # Grid lines
        for y in range(cy + 15, h, 25):
            draw.line([(0, y), (w, y)], fill=(0, 180, 200), width=1)

        # Crosshair Reticle
        cx = w // 2
        draw.ellipse([cx - 3, cy - 3, cx + 3, cy + 3], fill=(255, 204, 0))
        draw.line([(cx - 30, cy), (cx - 10, cy)], fill=(255, 204, 0), width=2)
        draw.line([(cx + 10, cy), (cx + 30, cy)], fill=(255, 204, 0), width=2)

        buf = io.BytesIO()
        img.save(buf, format='JPEG', quality=82)
        write_frame_to_stdout(buf.getvalue())

        target_time = start_time + (i + 1) / fps
        sleep_dur = target_time - time.time()
        if sleep_dur > 0:
            time.sleep(sleep_dur)

def main():
    transports = ['udp', 'tcp']
    t_idx = 0
    while True:
        current_transport = transports[t_idx % len(transports)]
        success = False
        if TARGET_URL.startswith("http://") or TARGET_URL.startswith("https://"):
            success = stream_http_mjpeg(TARGET_URL)
        else:
            success = stream_rtsp_single(TARGET_URL, current_transport)
        
        if not success:
            sys.stderr.write("[RTSP Relay] RTSP hardware offline — Generating smooth FPV standby video stream...\n")
            generate_standby_frames(3.0)

        t_idx += 1
        time.sleep(0.2)

if __name__ == "__main__":
    main()

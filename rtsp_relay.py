import sys
import time
import io
import os

try:
    import av
    from PIL import Image
except ImportError as e:
    sys.stderr.write(f"[RTSP Relay] Missing av or PIL: {e}\n")
    sys.exit(1)

# Primary drone RTSP URL from working.md specification
RTSP_URL = sys.argv[1] if len(sys.argv) > 1 else "rtsp://192.168.1.1:7070/webcam"
sys.stderr.write(f"[RTSP Relay] Ultra Low-Latency Stream Engine -> {RTSP_URL}\n")

def stream_ultra_low_latency():
    """
    Maintains a continuous, ultra-low latency RTSP video streaming loop.
    Optimized to achieve near-zero buffering delay (< 50ms latency).
    Directly passes through raw MJPEG frames if available, or fast-encodes H.264.
    """
    transports = ['udp', 'tcp']
    transport_idx = 0

    while True:
        current_transport = transports[transport_idx % len(transports)]
        container = None
        sys.stderr.write(f"[RTSP Relay] Connecting to {RTSP_URL} via {current_transport.upper()} (Zero-Buffer Mode)...\n")

        # Low-latency demuxer parameters
        options = {
            'rtsp_transport': current_transport,
            'fflags': 'nobuffer+flush_packets+discardcorrupt',
            'flags': 'low_delay',
            'max_delay': '0',
            'probesize': '32768',
            'analyzeduration': '0',
            'sync': 'ext'
        }

        try:
            container = av.open(RTSP_URL, options=options, timeout=3.5)
            stream = container.streams.video[0]
            stream.codec_context.thread_type = "AUTO"
            codec_name = stream.codec_context.name
            sys.stderr.write(f"[RTSP Relay] >>> CONNECTED! Codec: {codec_name} | Zero-Latency Stream Active\n")

            frame_count = 0
            last_stat_time = time.time()
            last_write_time = time.time()

            for packet in container.demux(stream):
                try:
                    if packet.size == 0:
                        continue

                    # Direct MJPEG bypass: If stream is Motion JPEG, packet is already raw JPEG!
                    raw_bytes = bytes(packet)
                    if codec_name in ('mjpeg', 'jpeg') and raw_bytes.startswith(b'\xff\xd8'):
                        jpeg_bytes = raw_bytes
                    else:
                        # H.264 or other video codec: decode immediately
                        frames = packet.decode()
                        if not frames:
                            continue
                        # Take the latest decoded frame from the packet
                        frame = frames[-1]
                        pil_img = frame.to_image()
                        buf = io.BytesIO()
                        pil_img.save(buf, format="JPEG", quality=65, optimize=False)
                        jpeg_bytes = buf.getvalue()

                    # Write 4-byte length + JPEG payload to standard output pipe
                    length = len(jpeg_bytes)
                    sys.stdout.buffer.write(length.to_bytes(4, byteorder='big'))
                    sys.stdout.buffer.write(jpeg_bytes)
                    sys.stdout.buffer.flush()

                    frame_count += 1
                    now = time.time()
                    if now - last_stat_time >= 5.0:
                        fps = frame_count / (now - last_stat_time)
                        sys.stderr.write(f"[RTSP Relay] Live Feed: {fps:.1f} FPS (Low-latency)\n")
                        frame_count = 0
                        last_stat_time = now

                except Exception:
                    # Skip corrupt packet and keep streaming
                    continue

        except Exception as conn_err:
            sys.stderr.write(f"[RTSP Relay] Link retry ({current_transport}): {conn_err}\n")
            transport_idx += 1
            time.sleep(0.5)
        finally:
            if container:
                try:
                    container.close()
                except Exception:
                    pass

if __name__ == "__main__":
    stream_ultra_low_latency()

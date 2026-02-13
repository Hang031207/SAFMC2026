import socket
import asyncio

async def test_udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(5)
    try:
        sock.sendto(b"test", ("192.168.0.99", 14560))
        print("✅ UDP socket works - packet sent")
    except Exception as e:
        print(f"❌ UDP failed: {e}")
    finally:
        sock.close()

asyncio.run(test_udp())

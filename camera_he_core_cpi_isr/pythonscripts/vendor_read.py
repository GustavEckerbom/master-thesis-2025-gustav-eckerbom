import usb.core
import usb.util

print("🔍 Finding all USB devices...\n")

devs = list(usb.core.find(find_all=True))

if not devs:
    print("❌ No USB devices found!")
else:
    for d in devs:
        print(f"✅ Found: VID=0x{d.idVendor:04X}, PID=0x{d.idProduct:04X}")
        try:
            print(f"   Manufacturer: {usb.util.get_string(d, d.iManufacturer)}")
            print(f"   Product:      {usb.util.get_string(d, d.iProduct)}")
        except Exception as e:
            print(f"   [!] Could not get strings: {e}")

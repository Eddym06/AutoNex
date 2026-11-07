"""Apply the tested WRO high-frame-rate PLL profile to esp32-camera 2.0.16."""

from pathlib import Path
import sys


def main() -> int:
    project = Path(__file__).resolve().parents[1]
    driver = project / "managed_components" / "espressif__esp32-camera" / "sensors" / "ov3660.c"
    if not driver.is_file():
        print(f"OV3660 driver missing: {driver}", file=sys.stderr)
        return 1

    source = driver.read_text(encoding="utf-8")
    tested = """            // WRO high-FPS experiment: 16 MHz XCLK -> 84 MHz SYSCLK,
            // keep DVP PCLK at 12 MHz and use the S3 PSRAM-DMA path.
            ret = set_pll(sensor, false, 21, 1, 0, false, 0, true, 14);"""
    qvga_stock = """            //16MHz SYSCLK and 8MHz PCLK (10.25 FPS)
            ret = set_pll(sensor, false, 8, 1, 0, false, 2, true, 4);"""
    qqvga_stock = """            //32MHz SYSCLK and 8MHz PCLK (17.77 FPS)
            ret = set_pll(sensor, false, 8, 1, 0, false, 0, true, 8);"""

    # The QVGA experiment proved unstable; keep the stock QVGA timing. Since
    # its temporary profile used the same block as QQVGA, restore the first
    # occurrence (the QVGA branch precedes QQVGA in set_framesize()).
    if source.count(tested) > 1:
        source = source.replace(tested, qvga_stock, 1)

    if qvga_stock not in source:
        print(
            "Unsupported esp32-camera OV3660 source; expected the 2.0.16 QVGA PLL block. "
            "Check the component update before building.",
            file=sys.stderr,
        )
        return 2

    if qqvga_stock in source:
        source = source.replace(qqvga_stock, tested, 1)
    driver.write_text(source, encoding="utf-8", newline="\n")
    print("Applied stock OV3660 PLL for QVGA and WRO profile for QQVGA (84 MHz SYSCLK, 12 MHz PCLK).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

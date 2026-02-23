from asyncua import Client
import asyncio
import time
import statistics
from datetime import datetime
import json


class LatencyAnalyzer:
    def __init__(self, plc_url: str = "opc.tcp://localhost:4840"):
        self.plc_url = plc_url
        self.client = None
        self.test_write_node = None
        self.test_read_node = None

    async def connect(self):
        self.client = Client(self.plc_url)
        self.client.set_user("")
        self.client.set_password("")
        self.client.session_timeout = 60000
        await self.client.connect()
        print("✅ Connected to CODESYS OPC UA Server")

        try:
            self.test_write_node = self.client.get_node(
                "ns=4;s=|var|CODESYS Control Win V3 x64.Application.PLC_PRG.Pick_Request"
            )
            self.test_read_node = self.client.get_node(
                "ns=4;s=|var|CODESYS Control Win V3 x64.Application.PLC_PRG.Pick_Done"
            )

            await self.test_write_node.read_value()
            await self.test_read_node.read_value()
            print("✅ Nodes registered and validated")

        except Exception as e:
            print(f"❌ Node registration error: {e}")
            raise

    async def measure_write_read_latency(self, iterations: int = 100):
        latencies = []

        print("🚀 Starting latency test...\n")

        for i in range(iterations):
            try:
                # --- Write ---
                start_time = time.perf_counter()
                await self.test_write_node.write_value(True)

                # --- Wait PLC reaction ---
                await asyncio.sleep(0.01)  # 10ms

                # --- Read ---
                value = await self.test_read_node.read_value()

                round_trip_ms = (time.perf_counter() - start_time) * 1000
                latencies.append(round_trip_ms)

                # --- Reset ---
                await self.test_write_node.write_value(False)
                await asyncio.sleep(0.02)

                if (i + 1) % 20 == 0:
                    print(f"   {i+1}/{iterations} samples collected")

            except Exception as e:
                print(f"❌ Iteration {i} failed: {e}")

        return latencies

    async def analyze_latency_distribution(self):
        print("\n📊 === LATENCY ANALYSIS ===")
        print("Measuring OPC UA write-read round-trip latency...\n")

        latencies = await self.measure_write_read_latency(100)

        if not latencies:
            print("❌ No valid measurements collected")
            return None

        latencies_sorted = sorted(latencies)

        mean = statistics.mean(latencies_sorted)
        median = statistics.median(latencies_sorted)
        stdev = statistics.stdev(latencies_sorted) if len(latencies_sorted) > 1 else 0
        p95 = latencies_sorted[int(len(latencies_sorted) * 0.95)]
        p99 = latencies_sorted[int(len(latencies_sorted) * 0.99)]

        print("\n✅ Analysis Complete!")
        print("\n📈 Round-Trip Latency (Write → PLC → Read)")
        print(f"   Mean:    {mean:.2f} ms")
        print(f"   Median:  {median:.2f} ms")
        print(f"   StdDev:  {stdev:.2f} ms")
        print(f"   P95:     {p95:.2f} ms")
        print(f"   P99:     {p99:.2f} ms")
        print(f"   Min:     {min(latencies_sorted):.2f} ms")
        print(f"   Max:     {max(latencies_sorted):.2f} ms")

        outliers = [x for x in latencies_sorted if x > mean + 3 * stdev]
        print(f"\n⚠️ Outliers (>3σ): {len(outliers)}")

        report = {
            "timestamp": datetime.now().isoformat(),
            "plc_url": self.plc_url,
            "samples": len(latencies_sorted),
            "latency_ms": {
                "mean": mean,
                "median": median,
                "stdev": stdev,
                "p95": p95,
                "p99": p99,
                "min": min(latencies_sorted),
                "max": max(latencies_sorted),
            },
            "raw_samples_first_20": latencies_sorted[:20],
        }

        with open("latency_report.json", "w") as f:
            json.dump(report, f, indent=2)

        print("\n💾 Report saved to latency_report.json")

        return report


async def main():
    analyzer = LatencyAnalyzer()

    try:
        await analyzer.connect()
        await analyzer.analyze_latency_distribution()

    finally:
        if analyzer.client:
            await analyzer.client.disconnect()
            print("🔌 Disconnected cleanly")


if __name__ == "__main__":
    asyncio.run(main())
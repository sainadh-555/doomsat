"""Command-line interface (CLI) for running DOOMSat simulations, HIL, and dashboard."""

import argparse
import sys
import time

from doomsat.sim.orchestrator import MissionOrchestrator, SimulationConfig
from doomsat.sim.hil_runner import HILBenchtopRunner


def main() -> None:
    """CLI entry point for GARUDA-1U mission simulation."""
    parser = argparse.ArgumentParser(
        prog="doomsat",
        description="GARUDA-1U CubeSat Digital Twin & Mission Operations CLI",
    )
    subparsers = parser.add_subparsers(dest="subcommand", help="Available subcommands")

    # 1. Simulation subparser
    sim_parser = subparsers.add_parser("sim", help="Run full-orbit mission simulation")
    sim_parser.add_argument("--duration", type=float, default=5700.0, help="Simulation duration [s] (default: 5700s ~ 1 orbit)")
    sim_parser.add_argument("--dt", type=float, default=1.0, help="Time step [s] (default: 1.0s)")
    sim_parser.add_argument("--altitude", type=float, default=500000.0, help="Initial altitude [m] (default: 500 km)")
    sim_parser.add_argument("--mode", type=str, default="SIMULATION", choices=["SIMULATION", "HIL"], help="Radio mode")

    # 2. HIL subparser
    hil_parser = subparsers.add_parser("hil", help="Run Hardware-in-the-Loop benchtop test with physical ESP32")
    hil_parser.add_argument("--port", type=str, default=None, help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
    hil_parser.add_argument("--baud", type=int, default=115200, help="UART baud rate (default: 115200)")

    # 3. Dashboard subparser
    dash_parser = subparsers.add_parser("dashboard", help="Launch FastAPI Mission Control web server")
    dash_parser.add_argument("--host", type=str, default="0.0.0.0", help="Listen host (default: 0.0.0.0)")
    dash_parser.add_argument("--port", type=int, default=8000, help="Listen port (default: 8000)")

    args = parser.parse_args()

    if args.subcommand == "sim":
        config = SimulationConfig(
            duration_s=args.duration,
            dt_s=args.dt,
            altitude_m=args.altitude,
            radio_mode=args.mode,
        )
        print(f"[*] Initializing GARUDA-1U Mission Simulator (Duration: {args.duration}s, dt: {args.dt}s)...")
        orchestrator = MissionOrchestrator(config)

        start_wall = time.time()
        snapshots = orchestrator.run()
        elapsed_wall = time.time() - start_wall

        print(f"[+] Simulation completed successfully in {elapsed_wall:.2f}s wall-clock time.")
        print(f"    Total simulated steps: {len(snapshots)}")
        print(f"    Final Battery SoC:     {snapshots[-1].battery_soc_pct:.1f}%")
        print(f"    Final Angular Speed:   {snapshots[-1].omega_norm_deg_s:.3f} deg/s")
        print(f"    Final Flight Mode:     {snapshots[-1].flight_mode}")

    elif args.subcommand == "hil":
        print("[*] Launching Hardware-in-the-Loop benchtop runner...")
        runner = HILBenchtopRunner(port=args.port, baud_rate=args.baud)
        ports = runner.list_available_ports()
        print(f"[*] Detected serial ports: {ports}")
        if not ports and not args.port:
            print("[-] No serial COM ports detected. Please connect an ESP32 board or specify --port.")
            sys.exit(1)
        if runner.connect():
            print(f"[+] Connected to ESP32 on {runner.port}. Testing CSTP PING...")
            success = runner.send_ping()
            print(f"[+] PING sent: {'SUCCESS' if success else 'FAILED'}")
            runner.close()

    elif args.subcommand == "dashboard":
        import uvicorn
        from doomsat.dashboard.app import app
        print(f"[*] Starting Mission Control Dashboard on http://{args.host}:{args.port}...")
        uvicorn.run(app, host=args.host, port=args.port)

    else:
        parser.print_help()


if __name__ == "__main__":
    main()

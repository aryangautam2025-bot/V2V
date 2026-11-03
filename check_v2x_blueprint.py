#!/usr/bin/env python3
"""Print runtime CARLA custom-V2X blueprint fields and test setting radio propagation parameters."""
import argparse
import carla


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2500)
    p.add_argument("--channel", default="v2v_coop_drone_mesh")
    args = p.parse_args()
    client = carla.Client(args.host, args.port)
    client.set_timeout(8.0)
    world = client.get_world()
    bp = world.get_blueprint_library().find("sensor.other.v2x_custom")
    print("CARLA map:", world.get_map().name)
    print("Blueprint:", bp.id)
    available = {}
    for attr in bp:
        key = getattr(attr, "id", str(attr))
        try:
            val = attr.as_string()
        except Exception:
            val = str(attr)
        available[key] = val
    for key in sorted(available):
        print(f"  {key} = {available[key]}")
    requested = {"channel_id": args.channel, "scenario": "urban", "path_loss_model": "geometric",
                 "filter_distance": "250", "use_etsi_fading": "true", "transmit_power": "21.5",
                 "receiver_sensitivity": "-99.0"}
    print("\nSET TEST:")
    for key, value in requested.items():
        if not bp.has_attribute(key):
            print(f"  {key}: MISSING (attribute not exposed)")
            continue
        try:
            bp.set_attribute(key, value)
            print(f"  {key}: SET -> {value}")
        except Exception as exc:
            print(f"  {key}: REJECTED -> {exc}")
    core = ("channel_id", "scenario", "path_loss_model")
    missing = [key for key in core if not bp.has_attribute(key)]
    if not missing:
        print("\nCore radio attributes are exposed. The dashboard should be able to configure the requested model.")
        print("This does not prove every individual packet took an RF-NLOS path; CARLA doesn't expose a definitive per-packet NLOS flag here.")
    else:
        print("\nCore attributes missing:", ", ".join(missing))
        print("Do not label radio NLOS active until the local CARLA build can configure these fields.")


if __name__ == "__main__":
    main()

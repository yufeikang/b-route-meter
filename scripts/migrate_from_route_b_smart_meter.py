#!/usr/bin/env python3
"""Migrate HA storage from route_b_smart_meter to b_route_meter.

Run this script AFTER stopping Home Assistant and BEFORE starting it
with the new b_route_meter component.

Usage:
    python3 migrate_from_route_b_smart_meter.py /path/to/hass/config

This modifies three files in .storage/:
  - core.config_entries: change domain
  - core.entity_registry: change platform
  - core.device_registry: change identifiers
"""

import json
import shutil
import sys
from pathlib import Path

OLD_DOMAIN = "route_b_smart_meter"
NEW_DOMAIN = "b_route_meter"


def migrate_config_entries(storage_dir: Path) -> int:
    path = storage_dir / "core.config_entries"
    data = json.loads(path.read_text())
    count = 0
    for entry in data["data"]["entries"]:
        if entry.get("domain") == OLD_DOMAIN:
            entry["domain"] = NEW_DOMAIN
            entry["title"] = entry["title"].replace("Route B Smart Meter", "B-Route Smart Meter")
            count += 1
    if count:
        path.write_text(json.dumps(data, indent=2, ensure_ascii=False))
    return count


def migrate_entity_registry(storage_dir: Path) -> int:
    path = storage_dir / "core.entity_registry"
    data = json.loads(path.read_text())
    count = 0
    for entity in data["data"]["entities"]:
        if entity.get("platform") == OLD_DOMAIN:
            entity["platform"] = NEW_DOMAIN
            count += 1
    if count:
        path.write_text(json.dumps(data, indent=2, ensure_ascii=False))
    return count


def migrate_device_registry(storage_dir: Path) -> int:
    path = storage_dir / "core.device_registry"
    data = json.loads(path.read_text())
    count = 0
    for device in data["data"]["devices"]:
        new_ids = []
        changed = False
        for identifier in device.get("identifiers", []):
            if isinstance(identifier, list) and len(identifier) == 2 and identifier[0] == OLD_DOMAIN:
                new_ids.append([NEW_DOMAIN, identifier[1]])
                changed = True
            else:
                new_ids.append(identifier)
        if changed:
            device["identifiers"] = new_ids
            count += 1
    if count:
        path.write_text(json.dumps(data, indent=2, ensure_ascii=False))
    return count


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} /path/to/hass/config")
        sys.exit(1)

    config_dir = Path(sys.argv[1])
    storage_dir = config_dir / ".storage"

    if not storage_dir.exists():
        print(f"Error: {storage_dir} not found")
        sys.exit(1)

    # Backup
    for name in ["core.config_entries", "core.entity_registry", "core.device_registry"]:
        src = storage_dir / name
        dst = storage_dir / f"{name}.bak"
        if src.exists() and not dst.exists():
            shutil.copy2(src, dst)
            print(f"Backed up {name} -> {name}.bak")

    # Migrate
    n1 = migrate_config_entries(storage_dir)
    print(f"Config entries: migrated {n1} entries")

    n2 = migrate_entity_registry(storage_dir)
    print(f"Entity registry: migrated {n2} entities")

    n3 = migrate_device_registry(storage_dir)
    print(f"Device registry: migrated {n3} devices")

    if n1 + n2 + n3 == 0:
        print("\nNothing to migrate. Already migrated or no route_b_smart_meter entries found.")
    else:
        print(f"\nDone! Migrated {n1} config entries, {n2} entities, {n3} devices.")
        print("You can now start Home Assistant with the b_route_meter component.")


if __name__ == "__main__":
    main()

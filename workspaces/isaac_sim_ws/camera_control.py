#!/usr/bin/env python3
import argparse
import json
import os
import uuid
from datetime import datetime
from pathlib import Path


DEFAULT_COMMAND_ROOT = Path(__file__).resolve().parent / "camera_commands"


def write_command(args):
    command_root = Path(args.command_root)
    inbox = command_root / "inbox"
    inbox.mkdir(parents=True, exist_ok=True)

    request_id = (
        datetime.now().strftime("%Y%m%d_%H%M%S")
        + "_"
        + uuid.uuid4().hex[:8]
    )
    command = {
        "id": request_id,
        "action": "save_depth",
        "camera": args.camera,
        "frames": args.frames,
        "every": args.every,
    }
    target = inbox / f"{request_id}.json"
    temporary = inbox / f".{request_id}.tmp"
    temporary.write_text(
        json.dumps(command, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, target)

    print(f"Запрос отправлен: {request_id}")
    print(f"Статус: {command_root / 'status' / (request_id + '.json')}")


def show_status(args):
    status_dir = Path(args.command_root) / "status"
    if args.request_id:
        status_path = status_dir / f"{args.request_id}.json"
    else:
        statuses = sorted(
            status_dir.glob("*.json"),
            key=lambda path: path.stat().st_mtime,
        )
        if not statuses:
            raise SystemExit("Статусы пока отсутствуют")
        status_path = statuses[-1]

    if not status_path.is_file():
        raise SystemExit(f"Статус не найден: {status_path}")
    print(status_path.read_text(encoding="utf-8"), end="")


def build_parser():
    parser = argparse.ArgumentParser(
        description="Управление записью depth-кадров работающего Isaac Sim"
    )
    parser.add_argument(
        "--command-root",
        default=str(DEFAULT_COMMAND_ROOT),
        help="папка очереди команд",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    save_parser = subparsers.add_parser("save", help="сохранить depth-кадры")
    save_parser.add_argument("--camera", required=True, help="имя камеры из YAML")
    save_parser.add_argument("--frames", type=int, default=10)
    save_parser.add_argument(
        "--every",
        type=int,
        default=1,
        help="сохранять каждый N-й кадр",
    )
    save_parser.set_defaults(handler=write_command)

    status_parser = subparsers.add_parser("status", help="показать статус")
    status_parser.add_argument("request_id", nargs="?")
    status_parser.set_defaults(handler=show_status)
    return parser


def main():
    args = build_parser().parse_args()
    if getattr(args, "frames", 1) < 1 or getattr(args, "every", 1) < 1:
        raise SystemExit("--frames и --every должны быть больше нуля")
    args.handler(args)


if __name__ == "__main__":
    main()

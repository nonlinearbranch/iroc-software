from __future__ import annotations

import time
from pathlib import Path
from typing import Any

from flask import Flask, jsonify, request

from iroc.comms.validation import validate_report
from iroc.config import CommsConfig, VisionConfig
from iroc.storage import atomic_write_json, ensure_dir, sha256_file


def create_app(config: CommsConfig, vision: VisionConfig | None = None) -> Flask:
    app = Flask(__name__)
    vision_config = vision or VisionConfig()
    root = ensure_dir(config.transfer_dir)
    seed_dir = ensure_dir(root / "seeds")
    report_dir = ensure_dir(root / "reports")
    image_dir = ensure_dir(root / "images")
    command_file = root / "command.json"

    @app.get("/health")
    def health():
        return jsonify({"ok": True, "time_s": time.time()})

    @app.post("/command/start")
    def start_command():
        payload: dict[str, Any] = request.get_json(silent=True) or {}
        payload.setdefault("command", "start")
        payload.setdefault("time_s", time.time())
        atomic_write_json(command_file, payload)
        return jsonify({"ok": True, "command": payload})

    @app.get("/command/latest")
    def latest_command():
        if not command_file.exists():
            return jsonify({"command": "idle"})
        return app.response_class(command_file.read_text(encoding="utf-8"), mimetype="application/json")

    @app.post("/seed/<name>")
    def upload_seed(name: str):
        if "file" not in request.files:
            return jsonify({"ok": False, "error": "missing file form field"}), 400
        file = request.files["file"]
        suffix = Path(file.filename or "").suffix or ".jpg"
        output = seed_dir / f"{Path(name).stem}{suffix}"
        file.save(output)
        return jsonify({"ok": True, "path": str(output), "sha256": sha256_file(output)})

    @app.get("/seeds")
    def list_seeds():
        files = [
            {"name": path.stem, "filename": path.name, "path": str(path), "sha256": sha256_file(path)}
            for path in sorted(seed_dir.iterdir())
            if path.is_file()
        ]
        return jsonify({"ok": True, "seeds": files})

    @app.get("/seed/<name>/download")
    def download_seed(name: str):
        matches = [path for path in sorted(seed_dir.iterdir()) if path.is_file() and path.stem == Path(name).stem]
        if not matches:
            return jsonify({"ok": False, "error": "seed not found"}), 404
        return app.response_class(matches[0].read_bytes(), mimetype="application/octet-stream")

    @app.post("/upload/report")
    def upload_report():
        payload = request.get_json(force=True)
        mission_id = str(payload.get("mission_id", f"mission_{int(time.time())}"))
        output = atomic_write_json(report_dir / f"{mission_id}.json", payload)
        atomic_write_json(report_dir / "latest.json", payload)
        return jsonify({"ok": True, "path": str(output), "sha256": sha256_file(output)})

    @app.get("/reports/latest")
    def latest_report():
        latest = report_dir / "latest.json"
        if not latest.exists():
            return jsonify({"ok": False, "error": "no report"}), 404
        return app.response_class(latest.read_text(encoding="utf-8"), mimetype="application/json")

    @app.post("/validate/latest")
    def validate_latest():
        latest = report_dir / "latest.json"
        if not latest.exists():
            return jsonify({"ok": False, "error": "no latest report"}), 404
        try:
            result = validate_report(
                seed_dir=seed_dir,
                report_path=latest,
                image_dir=image_dir,
                vision=vision_config,
                out_path=report_dir / "latest_validation.json",
            )
        except Exception as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400
        return jsonify({"ok": True, "validation": result})

    @app.post("/upload/image/<filename>")
    def upload_image(filename: str):
        data = request.get_data()
        if not data:
            return jsonify({"ok": False, "error": "empty body"}), 400
        safe_name = Path(filename).name
        output = image_dir / safe_name
        output.write_bytes(data)
        return jsonify({"ok": True, "path": str(output), "sha256": sha256_file(output)})

    return app


def run_base_station(config: CommsConfig, vision: VisionConfig | None = None) -> None:
    app = create_app(config, vision)
    app.run(host=config.bind_host, port=config.port)

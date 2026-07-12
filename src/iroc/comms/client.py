from __future__ import annotations

import json
import mimetypes
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any

from iroc.config import CommsConfig


class BaseStationClient:
    def __init__(self, config: CommsConfig) -> None:
        self.config = config
        self.base_url = config.base_url.rstrip("/")

    def health(self) -> dict[str, Any]:
        return self._get_json("/health")

    def latest_command(self) -> dict[str, Any]:
        return self._get_json("/command/latest")

    def list_seeds(self) -> list[dict[str, Any]]:
        payload = self._get_json("/seeds")
        return list(payload.get("seeds", []))

    def download_seeds(self, output_dir: str | Path) -> list[Path]:
        directory = Path(output_dir)
        directory.mkdir(parents=True, exist_ok=True)
        written: list[Path] = []
        for seed in self.list_seeds():
            name = str(seed["name"])
            filename = str(seed.get("filename") or f"{name}.jpg")
            req = urllib.request.Request(
                self.base_url + "/seed/" + urllib.parse.quote(name) + "/download",
                method="GET",
            )
            try:
                with urllib.request.urlopen(req, timeout=self.config.timeout_s) as response:
                    data = response.read()
            except urllib.error.URLError as exc:
                raise ConnectionError(f"Seed download failed for {name}: {exc}") from exc
            path = directory / Path(filename).name
            path.write_bytes(data)
            written.append(path)
        if not written:
            raise RuntimeError("Base station returned no seed images")
        return written

    def upload_report(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._post_json("/upload/report", payload)

    def upload_image(self, path: str | Path) -> dict[str, Any]:
        image_path = Path(path)
        mime = mimetypes.guess_type(image_path.name)[0] or "application/octet-stream"
        req = urllib.request.Request(
            self.base_url + "/upload/image/" + urllib.parse.quote(image_path.name),
            data=image_path.read_bytes(),
            method="POST",
            headers={"Content-Type": mime},
        )
        return self._read_json(req)

    def _get_json(self, path: str) -> dict[str, Any]:
        req = urllib.request.Request(self.base_url + path, method="GET")
        return self._read_json(req)

    def _post_json(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            self.base_url + path,
            data=data,
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        return self._read_json(req)

    def _read_json(self, req: urllib.request.Request) -> dict[str, Any]:
        try:
            with urllib.request.urlopen(req, timeout=self.config.timeout_s) as response:
                return json.loads(response.read().decode("utf-8"))
        except urllib.error.URLError as exc:
            raise ConnectionError(f"Base station request failed: {exc}") from exc

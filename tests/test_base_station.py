import io

import cv2
import numpy as np

from iroc.comms.base_station import create_app
from iroc.config import CommsConfig, VisionConfig


def test_base_station_accepts_seed_command_and_report(tmp_path):
    config = CommsConfig(transfer_dir=str(tmp_path))
    app = create_app(config)
    client = app.test_client()

    health = client.get("/health")
    assert health.status_code == 200
    assert health.get_json()["ok"]

    seed = client.post(
        "/seed/sample",
        data={"file": (io.BytesIO(b"fake-image-bytes"), "sample.jpg")},
        content_type="multipart/form-data",
    )
    assert seed.status_code == 200
    assert seed.get_json()["ok"]
    seed_list = client.get("/seeds")
    assert seed_list.status_code == 200
    assert seed_list.get_json()["seeds"][0]["name"] == "sample"
    seed_download = client.get("/seed/sample/download")
    assert seed_download.status_code == 200
    assert seed_download.data == b"fake-image-bytes"

    command = client.post("/command/start", json={"mission": "demo"})
    assert command.status_code == 200
    assert command.get_json()["ok"]

    report = client.post("/upload/report", json={"mission_id": "m1", "detections": []})
    assert report.status_code == 200
    assert report.get_json()["ok"]
    assert (tmp_path / "reports" / "m1.json").exists()


def test_base_station_validates_uploaded_evidence(tmp_path):
    vision = VisionConfig(min_good_matches=8, min_inliers=4, min_score=0.2, methods=("area", "lanczos"))
    app = create_app(CommsConfig(transfer_dir=str(tmp_path)), vision)
    client = app.test_client()

    rng = np.random.default_rng(4)
    image = rng.integers(0, 255, (220, 220, 3), dtype=np.uint8)
    cv2.putText(image, "V", (70, 145), cv2.FONT_HERSHEY_SIMPLEX, 3.0, (255, 255, 255), 5)
    ok, encoded = cv2.imencode(".png", image)
    assert ok

    seed = client.post(
        "/seed/sample",
        data={"file": (io.BytesIO(encoded.tobytes()), "sample.png")},
        content_type="multipart/form-data",
    )
    assert seed.status_code == 200

    upload = client.post("/upload/image/evidence.png", data=encoded.tobytes(), content_type="image/png")
    assert upload.status_code == 200

    report = client.post(
        "/upload/report",
        json={"mission_id": "m2", "detections": [{"seed_name": "sample", "image_path": "evidence.png"}]},
    )
    assert report.status_code == 200

    validation = client.post("/validate/latest")
    assert validation.status_code == 200
    payload = validation.get_json()
    assert payload["ok"]
    assert payload["validation"]["valid"]

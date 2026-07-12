from iroc.config import PowerConfig
from iroc.power.monitor import SimPowerMonitor, charging_confirmed


def test_sim_power_monitor_confirms_charging():
    config = PowerConfig(mode="sim")
    monitor = SimPowerMonitor(config)
    monitor.connect()

    initial = monitor.read_status()
    status = monitor.read_status()

    assert status.contact_detected
    assert status.charging
    assert charging_confirmed(status, config, initial.soc_pct)

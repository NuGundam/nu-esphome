"""Reserve the mDNS service slot that a -DUSE_WEBSERVER build flag needs.

ESPHome sizes its mDNS service list from the components in the config. A device with a
custom web page and the USE_WEBSERVER flags (for HA's Visit link) has no web_server
component, so one slot is reserved for two services. StaticVector::emplace_next() then
hands back the last slot and the _http service overwrites _esphomelib, which hides the
device's version and config_hash from the Device Builder.

Usage:
  external_components:
    - source:
        type: local
        path: components
      components: [mdns_web_slot]
  mdns_web_slot:
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.core import CORE
from esphome.coroutine import CoroPriority, coroutine_with_priority

DEPENDENCIES = ["mdns"]
CONFIG_SCHEMA = cv.Schema({})


@coroutine_with_priority(CoroPriority.LATE)  # after mdns (NETWORK_SERVICES) has set the count
async def to_code(config):
    current = [d for d in CORE.defines if d.name == "MDNS_SERVICE_COUNT"]
    if len(current) != 1:
        raise cv.Invalid("mdns_web_slot: expected exactly one MDNS_SERVICE_COUNT define")
    count = int(str(current[0].value))
    CORE.defines.discard(current[0])
    cg.add_define("MDNS_SERVICE_COUNT", count + 1)

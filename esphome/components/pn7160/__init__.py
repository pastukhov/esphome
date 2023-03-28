from esphome import automation, pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import spi, nfc
from esphome.const import CONF_ID, CONF_ON_TAG_REMOVED, CONF_ON_TAG, CONF_TRIGGER_ID

AUTO_LOAD = ["nfc"]
CODEOWNERS = ["@kbx81", "@jesserockz"]
DEPENDENCIES = ["spi"]

CONF_DWL_REQ_PIN = "dwl_req_pin"
CONF_IRQ_PIN = "irq_pin"
CONF_ON_FINISHED_WRITE = "on_finished_write"
CONF_ON_EMULATED_TAG_SCAN = "on_emulated_tag_scan"
CONF_PN7160_ID = "pn7160_id"
CONF_TAG_TTL = "tag_ttl"
CONF_VEN_PIN = "ven_pin"
CONF_WKUP_REQ_PIN = "wkup_req_pin"

pn7160_ns = cg.esphome_ns.namespace("pn7160")
PN7160 = pn7160_ns.class_("PN7160", cg.PollingComponent, spi.SPIDevice)


PN7160OnEmulatedTagScanTrigger = pn7160_ns.class_(
    "PN7160OnEmulatedTagScanTrigger", automation.Trigger.template()
)

PN7160OnFinishedWriteTrigger = pn7160_ns.class_(
    "PN7160OnFinishedWriteTrigger", automation.Trigger.template()
)

PN7160IsWritingCondition = pn7160_ns.class_(
    "PN7160IsWritingCondition", automation.Condition
)


IsWritingCondition = nfc.nfc_ns.class_("IsWritingCondition", automation.Condition)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(PN7160),
            cv.Optional(CONF_ON_EMULATED_TAG_SCAN): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        PN7160OnEmulatedTagScanTrigger
                    ),
                }
            ),
            cv.Optional(CONF_ON_FINISHED_WRITE): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        PN7160OnFinishedWriteTrigger
                    ),
                }
            ),
            cv.Optional(CONF_ON_TAG): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(nfc.NfcOnTagTrigger),
                }
            ),
            cv.Optional(CONF_ON_TAG_REMOVED): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(nfc.NfcOnTagTrigger),
                }
            ),
            cv.Optional(CONF_TAG_TTL): cv.positive_time_period_milliseconds,
            cv.Required(CONF_DWL_REQ_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_IRQ_PIN): pins.gpio_input_pin_schema,
            cv.Required(CONF_VEN_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_WKUP_REQ_PIN): pins.gpio_output_pin_schema,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema(cs_pin_required=True))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    pin = await cg.gpio_pin_expression(config[CONF_DWL_REQ_PIN])
    cg.add(var.set_dwl_req_pin(pin))

    pin = await cg.gpio_pin_expression(config[CONF_IRQ_PIN])
    cg.add(var.set_irq_pin(pin))

    pin = await cg.gpio_pin_expression(config[CONF_VEN_PIN])
    cg.add(var.set_ven_pin(pin))

    pin = await cg.gpio_pin_expression(config[CONF_WKUP_REQ_PIN])
    cg.add(var.set_wkup_req_pin(pin))

    if CONF_TAG_TTL in config:
        cg.add(var.set_tag_ttl(config[CONF_TAG_TTL]))

    for conf in config.get(CONF_ON_TAG, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        cg.add(var.register_ontag_trigger(trigger))
        await automation.build_automation(
            trigger, [(cg.std_string, "x"), (nfc.NfcTag, "tag")], conf
        )

    for conf in config.get(CONF_ON_TAG_REMOVED, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        cg.add(var.register_ontagremoved_trigger(trigger))
        await automation.build_automation(
            trigger, [(cg.std_string, "x"), (nfc.NfcTag, "tag")], conf
        )

    for conf in config.get(CONF_ON_EMULATED_TAG_SCAN, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)

    for conf in config.get(CONF_ON_FINISHED_WRITE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [], conf)


@automation.register_condition(
    "pn7160.is_writing",
    PN7160IsWritingCondition,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(PN7160),
        }
    ),
)
async def pn7160_is_writing_to_code(config, condition_id, template_arg, args):
    var = cg.new_Pvariable(condition_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var

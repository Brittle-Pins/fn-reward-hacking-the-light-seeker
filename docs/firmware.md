# Reward Hacking: The Self-Fulfilling Light Seeker. Firmware

## Open questions

1. Q-table persistence: Should the learned weights be saved to the EEPROM? The answer may depend on the duration of the training process.
2. Weights persistence: The rotary encoder allows for precise control of the input values. However, due to the infinite physical rotation the boundaries must be enforced in software. Moreover, upon a power cycle it loses its state unless saved to non-volatile memory. Should the weights be saved to the EEPROM?

// SPDX-License-Identifier: GPL-2.0
#include "ec.h"

u8 *ec_command_data(struct gaokun_ec *ec, u8 mcmd, u8 scmd, u8 ilen, u8 *buf, u8 olen)
{
	u8 ibuf[EC_INPUT_BUFFER_LENGTH];
	static u8 obuf[EC_OUTPUT_BUFFER_LENGTH];
	int ret;

	mutex_lock(&ec->lock);

	ibuf[0] = scmd;
	ibuf[1] = ilen;

	if (ilen > EC_INPUT_BUFFER_LENGTH || olen > EC_OUTPUT_BUFFER_LENGTH) { /* overflow */
		obuf[0] = 0x02; // ACPI do this, intention?
		goto err;
	}

	if(ilen > 0){
		// copy the data block
		memcpy(ibuf + 2, buf, ilen);
	}
	else{
		ibuf[2] = 0; // ACPI use ibuf[3] = buf, may be unnecessary.
	}

	switch(ilen) {
	case 0: case 1: case 2: case 3: case 4: case 5:
	case 0x18:
		ret = i2c_smbus_write_i2c_block_data(ec->client, mcmd, ilen + 2, ibuf);
		if (ret) {
			dev_err(&ec->client->dev, "I2C EC write failed with error code: %d\n", ret);
			goto err;
		}
		break;
	default:
		// ACPI allow this, so we don't goto err.
		dev_warn(&ec->client->dev, "Unsupported input data buffer length: %d\n", ilen);
	}

	usleep_range(2500, 3000); // Sleep (0x02)


	switch(olen) {
		case 1: case 2: case 3: case 4: case 5: case 6:
		case 7: case 8: case 0xF: case 0x16: case 0x20:
		case 0x23: case 0x40: case 0xFE: case 0xFF: case 0x9:
			ret = i2c_smbus_read_i2c_block_data(ec->client, mcmd, olen, obuf);
			if (ret < 0) {
				dev_err(&ec->client->dev, "I2C EC read failed with error code: %d\n", ret);
			}else{
				dev_info(&ec->client->dev, "I2C EC read successful, data: %*ph\n", olen, obuf);
			}
			break;
		default:
			dev_warn(&ec->client->dev, "Unsupported output data buffer length: %d\n", olen);
	}

err:
	mutex_unlock(&ec->lock);
	return obuf;
}

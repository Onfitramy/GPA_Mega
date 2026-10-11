#include "SPI2Queue.h"

#include <stdlib.h>
#include <string.h>

#include "Buzzer.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "SD.h"
#include "W25Q1.h"

extern QueueHandle_t SPI2Queue;
extern volatile W25QPage0_config_t W25Q_FLASH_CONFIG;

SPI2_ActionPacket_t SPI2_CreatePacket(SPI2_Action_t action, SPI2_ActionPayload_u payload) {
	SPI2_ActionPacket_t action_packet;
	action_packet.action = action;
	action_packet.payload = payload;

	return action_packet;
}

uint8_t SPI2_QueueAction(SPI2_ActionPacket_t action_packet) {
	// For bools 1 is true, but we want 0 when everything is fine.
	//
	// We create another pointer of action_packet
	// because FreeRTOS wants a pointer to the item we want to send,
	// which is a pointer.
	return xQueueSend(SPI2Queue, &action_packet, 0) != pdPASS;
}

uint8_t SPI2_QueueWrite(uint8_t* data, uint32_t size, SPI2_W25Q_WriteType_t type) {
	SPI2_ActionPayload_u payload;
	payload.write.data = malloc(size);
	memcpy(payload.write.data, data, size);
	payload.write.size = size;
	payload.write.type = type;

	SPI2_ActionPacket_t action_packet = SPI2_CreatePacket(SPI2_Action_W25Q_Write, payload);

	return SPI2_QueueAction(action_packet);
}

/**
 * Queue a read action.
 * The finished pointer of `read_data` will be set to false when this function is called.
 * When the reading is finished, it changes to true.
 * The packets field of `read_data` can than be accessed.
 */
uint8_t SPI2_QueueRead(SPI2_W25Q_Read_t* read_data) {
	read_data->finished = false;

	SPI2_ActionPayload_u payload;
	payload.read = read_data;

	SPI2_ActionPacket_t action_packet = SPI2_CreatePacket(SPI2_Action_W25Q_Read, payload);

	return SPI2_QueueAction(action_packet);
}

uint8_t SPI2_QueueErase(uint32_t first_sector, uint32_t last_sector) {
	SPI2_ActionPayload_u payload;
	payload.erase.first_sector = first_sector;
	payload.erase.last_sector = last_sector;

	SPI2_ActionPacket_t action_packet = SPI2_CreatePacket(SPI2_Action_W25Q_Erase, payload);

	return SPI2_QueueAction(action_packet);
}

uint8_t SPI2_QueueSetWriteStatus(bool write_logs) {
	SPI2_ActionPayload_u payload;
	payload.write_status.write_logs = write_logs;

	SPI2_ActionPacket_t action_packet = SPI2_CreatePacket(SPI2_Action_W25Q_SetWriteStatus, payload);

	return SPI2_QueueAction(action_packet);
}

uint8_t SPI2_QueueWriteConfig() {
	SPI2_ActionPayload_u payload;
	SPI2_ActionPacket_t action_packet = SPI2_CreatePacket(SPI2_Action_W25Q_WriteConfig, payload);

	return SPI2_QueueAction(action_packet);
}

uint8_t SPI2_QueueCopyLogs(uint16_t max_page, SPI2_W25Q_CopyLogsTarget_t target) {
	SPI2_ActionPayload_u payload;
	payload.copy_logs.max_page = max_page;
	payload.copy_logs.target = target;

	SPI2_ActionPacket_t action_packet = SPI2_CreatePacket(SPI2_Action_W25Q_CopyLogs, payload);

	return SPI2_QueueAction(action_packet);
}

uint8_t SPI2_QueueMountOperation(SPI2_SD_MountType_t type) {
	SPI2_ActionPayload_u payload;
	payload.mount_operation.type = type;

	SPI2_ActionPacket_t action_packet = SPI2_CreatePacket(SPI2_Action_SD_MountOperation, payload);

	return SPI2_QueueAction(action_packet);
}

uint8_t SPI2_QueueSelfTest(SPI2_SD_SelfTest_t* self_test_data) {
	self_test_data->finished = false;

	SPI2_ActionPayload_u payload;
	payload.self_test = self_test_data;

	SPI2_ActionPacket_t action_packet = SPI2_CreatePacket(SPI2_Action_SD_SelfTest, payload);

	return SPI2_QueueAction(action_packet);
}

/**
 * Queue a read action and block until the action is finished.
 * For more information see `W25Q_QueueRead`.
 */
uint8_t SPI2_QueueBlockingRead(SPI2_W25Q_Read_t* read_data) {
	uint8_t result = SPI2_QueueRead(read_data);

	if (result != 0) {
		return result;
	}

	while (!read_data->finished) {
		// If this function is called from a higher priority task than the flash task,
		// a while loop without any delays would cause the flash task to be never executed.
		// This would result in an infinite loop, which we don't really want.
		vTaskDelay(1);
	}

	return result;
}

void SPI2_HandleAction(SPI2_ActionPacket_t action_packet) {
	switch (action_packet.action) {
	case SPI2_Action_W25Q_Write: {
		SPI2_W25Q_Write_t write_data = action_packet.payload.write;
		switch (write_data.type) {
		case SPI2_WriteType_DataPackets: {
			DataPacket_t* packets = (DataPacket_t*) write_data.data;
			W25Q_WritePackets(packets, write_data.size / sizeof(DataPacket_t));
			break;
		}
		case SPI2_WriteType_Raw:
			W25Q_SaveToLog(write_data.data, write_data.size);
			W25Q_WriteConfig();
			break;
		}

		// The data to be written lives on the heap.
		free(write_data.data);

		break;
	}
	break;
	case SPI2_Action_W25Q_Read: {
		SPI2_W25Q_Read_t* read_data = action_packet.payload.read;
		uint8_t data[read_data->size];

		W25Q_Read(read_data->start_page, read_data->offset, read_data->size, data);

		memcpy(read_data->data, data, read_data->size);
		read_data->finished = true;
		break;
	}
	case SPI2_Action_W25Q_Erase: {
		SPI2_W25Q_Erase_t erase_data = action_packet.payload.erase;
		if (erase_data.first_sector == 0 && erase_data.last_sector == 0) {
			W25Q_Chip_Erase();
			buzzerPlayNote("C6", 100);
		} else if (erase_data.last_sector >= erase_data.first_sector) {
			for (uint32_t i = erase_data.first_sector; i <= erase_data.last_sector; ++i) {
				W25Q_Erase_Sector(i);
			}

			buzzerPlayNote("C6", 100);
		} else {
			// TODO: Error handling
		}
		break;
	}
	case SPI2_Action_W25Q_SetWriteStatus:
		W25Q_FLASH_CONFIG.write_logs = action_packet.payload.write_status.write_logs;
		W25Q_WriteConfig();
		break;
	case SPI2_Action_W25Q_WriteConfig:
		W25Q_WriteConfig();
		break;
	case SPI2_Action_W25Q_CopyLogs: {
		SPI2_W25Q_CopyLogs copy_logs_data = action_packet.payload.copy_logs;
		W25Q_CopyLogs(copy_logs_data.max_page, copy_logs_data.target);
		break;
	}
	case SPI2_Action_SD_MountOperation: {
		SPI2_SD_MountOperation_t mount_operation_data = action_packet.payload.mount_operation;
		switch (mount_operation_data.type) {
		case SPI2_SD_MountType_Mount:
			SD_Mount();
			break;
		case SPI2_SD_MountType_Unmount:
			SD_Unmount();
			break;
		}
		break;
	}
	case SPI2_Action_SD_SelfTest:
		SPI2_SD_SelfTest_t* self_test_data = action_packet.payload.self_test;
		self_test_data->result = SD_SelfTest();
		self_test_data->finished = true;
		break;
	}
}
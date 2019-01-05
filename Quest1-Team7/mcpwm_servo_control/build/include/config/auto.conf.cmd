deps_config := \
	/home/ChengWuliang/esp/esp-idf/components/app_trace/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/aws_iot/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/bt/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/driver/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/esp32/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/esp_adc_cal/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/esp_http_client/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/ethernet/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/fatfs/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/freertos/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/heap/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/http_server/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/libsodium/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/log/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/lwip/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/mbedtls/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/mdns/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/openssl/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/pthread/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/spi_flash/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/spiffs/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/tcpip_adapter/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/vfs/Kconfig \
	/home/ChengWuliang/esp/esp-idf/components/wear_levelling/Kconfig \
	/home/ChengWuliang/esp/esp-idf/Kconfig.compiler \
	/home/ChengWuliang/esp/esp-idf/components/bootloader/Kconfig.projbuild \
	/home/ChengWuliang/esp/esp-idf/components/esptool_py/Kconfig.projbuild \
	/home/ChengWuliang/esp/esp-idf/components/partition_table/Kconfig.projbuild \
	/home/ChengWuliang/esp/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;

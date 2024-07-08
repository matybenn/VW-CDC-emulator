idf_component_register(SRCS "vagComm.cpp"
                    "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES bt
                    REQUIRES driver
                    REQUIRES nvs_flash)
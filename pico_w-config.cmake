set(PICO_BOARD pico_w)

if (DEFINED ENV{PICO_PROBE_TX_PIN} AND (NOT PICO_PROBE_TX_PIN))
    set(PICO_PROBE_TX_PIN $ENV{PICO_PROBE_TX_PIN})
    message(STATUS  "Using PICO_PROBE_RX_PIN from environment ('${PICO_PROBE_RX_PIN}')")
else()
    set(PICO_PROBE_TX_PIN 0)
    message(STATUS "Using PICO_PROBE_TX_PIN (${PICO_PROBE_TX_PIN})")
endif ()
    
if (DEFINED ENV{PICO_PROBE_RX_PIN} AND (NOT PICO_PROBE_RX_PIN))
    set(PICO_PROBE_RX_PIN $ENV{PICO_PROBE_RX_PIN})
    message(STATUS "Using PICO_PROBE_RX_PIN from environment (${PICO_PROBE_RX_PIN})")
else()
    set(PICO_PROBE_RX_PIN 1)
    message(STATUS "Using PICO_PROBE_RX_PIN (${PICO_PROBE_RX_PIN})")
endif ()


# pull pico config
# Determine the user's home directory in a portable way
if(WIN32)
    set(USER_HOME_DIR "$ENV{USERPROFILE}")
    set(PICO_CONFIG_FILE_PATH "o:/.config/${PICO_BOARD}/pico_config.cmake")
else()
    set(USER_HOME_DIR "$ENV{HOME}")
    # Path to the external configuration file in a subfolder of the user's home directory
    set(PICO_CONFIG_FILE_PATH "${USER_HOME_DIR}/Obelix-Tresor/.config/${PICO_BOARD}/pico_config.cmake")
endif()


# Check if the external configuration file exists
if(EXISTS ${PICO_CONFIG_FILE_PATH})
    include(${PICO_CONFIG_FILE_PATH})
else()
    message(FATAL_ERROR "Pico project configuration file not found. Please create ${PICO_CONFIG_FILE_PATH}")
endif()
# FindMAVLink.cmake
# Busca los headers de MAVLink y define:
#   MAVLINK_INCLUDE_DIRS : rutas de inclusión
#   MAVLINK_VERSION      : versión encontrada

include(FindPackageHandleStandardArgs)

# Si no se define _MAVLINK_INCLUDE_DIR, se usa la ruta por defecto:
# Se asume que la carpeta MAVLink está en <tu_paquete>/mavlink/include/mavlink
if(NOT DEFINED _MAVLINK_INCLUDE_DIR)
  set(_MAVLINK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/mavlink/include/mavlink")
endif()

# Verifica que exista el archivo clave (por ejemplo, mavlink_types.h)
if(NOT EXISTS "${_MAVLINK_INCLUDE_DIR}/mavlink_types.h")
  message(FATAL_ERROR "No se encontró la carpeta de inclusión de MAVLink en: ${_MAVLINK_INCLUDE_DIR}. Por favor, define _MAVLINK_INCLUDE_DIR manualmente.")
endif()

# Usamos la ruta definida sin modificaciones adicionales
set(MAVLINK_INCLUDE_DIRS "${_MAVLINK_INCLUDE_DIR}")

# Fijamos la versión (puedes modificarla si es necesario)
set(MAVLINK_VERSION "2.0")

find_package_handle_standard_args(MAVLink
    REQUIRED_VARS MAVLINK_INCLUDE_DIRS MAVLINK_VERSION
    VERSION_VAR MAVLINK_VERSION
)

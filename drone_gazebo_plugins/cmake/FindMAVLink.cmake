# FindMAVLink.cmake
#
# Este módulo busca la librería header-only MAVLink.
# Se espera encontrar el archivo "mavlink_types.h" en la ruta de inclusión.
#
# Se definen las siguientes variables:
#  MAVLINK_FOUND        - Verdadero si se encontró MAVLink.
#  MAVLINK_INCLUDE_DIRS - Directorios de inclusión.
#  MAVLINK_VERSION      - Versión encontrada (si se puede determinar).

include(FindPackageHandleStandardArgs)

# Define algunas pistas de búsqueda. Ajusta estas rutas según tu instalación.
set(_MAVLINK_SEARCH_HINTS
  "$ENV{HOME}/Documents/GitHub/mavlink/install/include"  # Instalación generada con CMake.
  "$ENV{HOME}/Documents/GitHub/mavlink"                  # Repositorio clonado, si no se copió todo.
)

# Buscar el archivo "mavlink_types.h" en las rutas sugeridas.
find_path(_MAVLINK_INCLUDE_DIR
  NAMES mavlink_types.h
  HINTS ${_MAVLINK_SEARCH_HINTS}
  PATH_SUFFIXES mavlink mavlink/v2.0
)

# Si se encontró el archivo "mavlink/config.h", tratar de extraer la versión
if(EXISTS "${_MAVLINK_INCLUDE_DIR}/mavlink/config.h")
  file(READ "${_MAVLINK_INCLUDE_DIR}/mavlink/config.h" MAVLINK_CONFIG_FILE)
  string(REGEX MATCH "#define[ \t]+MAVLINK_VERSION[ \t]+\"(([0-9]+\\.)+[0-9]+)\""
         _MAVLINK_VERSION_MATCH "${MAVLINK_CONFIG_FILE}")
  set(MAVLINK_VERSION "${CMAKE_MATCH_1}")
else()
  # Si no se encontró, se asume una versión predeterminada (puedes ajustarla)
  set(MAVLINK_VERSION "2.0")
endif()

# Establecer las variables de salida
set(MAVLINK_INCLUDE_DIRS "${_MAVLINK_INCLUDE_DIR}")

find_package_handle_standard_args(
  MAVLink
  REQUIRED_VARS MAVLINK_INCLUDE_DIRS MAVLINK_VERSION
  VERSION_VAR MAVLINK_VERSION
)

mark_as_advanced(MAVLINK_INCLUDE_DIR MAVLINK_INCLUDE_DIRS)

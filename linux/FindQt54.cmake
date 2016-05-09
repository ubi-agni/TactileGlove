# http://www.kdab.com/using-cmake-with-qt-5

set(_all_components
  Core
  Gui
  DBus
  Designer
  Declarative
  Script
  ScriptTools
  Network
  Test
  Xml
  Svg
  Sql
  Widgets
  PrintSupport
  Concurrent
  UiTools
  WebKit
  WebKitWidgets
  OpenGL
  X11Extras
  Qml
  Quick
  )

find_package(Qt5Core QUIET)

if (Qt5Core_FOUND)
  message(STATUS "found Qt5: ${Qt5Core_DIR}")
  if (NOT Qt54_FIND_COMPONENTS)
	 foreach(_component ${_all_components})
		find_package(Qt5${_component})
		list(APPEND QT_LIBRARIES ${Qt5${_component}_LIBRARIES})
	 endforeach()
  else()
	 foreach(_component ${Qt54_FIND_COMPONENTS})
		find_package(Qt5${_component} REQUIRED)
		list(APPEND QT_LIBRARIES ${Qt5${_component}_LIBRARIES})

		if ("${_component}" STREQUAL "WebKit")
		  find_package(Qt5WebKitWidgets REQUIRED)
		  list(APPEND QT_LIBRARIES ${Qt5WebKitWidgets_LIBRARIES} )
		endif()
		if ("${_component}" STREQUAL "Gui")
		  find_package(Qt5Widgets REQUIRED)
		  list(APPEND QT_LIBRARIES ${Qt5Widgets_LIBRARIES} )
		endif()
		if ("${_component}" STREQUAL "Core")
		  find_package(Qt5Concurrent REQUIRED)
		  list(APPEND QT_LIBRARIES ${Qt5Concurrent_LIBRARIES} )
		endif()
	 endforeach()
  endif()

  LIST(APPEND QT_DEFINITIONS "-fPIC")
  set(QT_FOUND TRUE)
  set(QT5_BUILD TRUE)

else(Qt5Core_FOUND)

  foreach(_component ${Qt54_FIND_COMPONENTS})
	 if("${_component}" STREQUAL "Widgets")  # new in Qt5
		set(_component Gui)
	 elseif("${_component}" STREQUAL "Concurrent")   # new in Qt5
		set(_component Core)
	 endif()
	 list(APPEND _components Qt${_component})
  endforeach()
  find_package(Qt4 ${QT_MIN_VERSION} REQUIRED ${_components})

  if(QT4_FOUND)
	 set(QT_FOUND TRUE)
  endif()
endif()

# define version-agnostic macros and variables
if(QT5_BUILD)
  foreach(_module ${_all_components})
	 string(TOUPPER ${_module} _module_upper)
	 set(QT${_module_upper}_LIBRARIES ${Qt5${_module}_LIBRARIES})
	 set(QT${_module_upper}_LIBRARY ${QT_QT${_module_upper}_LIBRARIES})
	 list(APPEND QT_INCLUDES ${Qt5${_module}_INCLUDE_DIRS})
	 set(QT${_module_upper}_FOUND ${Qt5${_module}_FOUND})
  endforeach()

  macro(qt_wrap_ui)
	 qt5_wrap_ui(${ARGN})
  endmacro()

  macro(qt_wrap_cpp)
	 qt5_wrap_cpp(${ARGN})
  endmacro()

  macro(qt_add_dbus_adaptor)
	 qt5_add_dbus_adaptor(${ARGN})
  endmacro()

  macro(qt_add_dbus_interfaces)
	 qt5_add_dbus_interfaces(${ARGN})
  endmacro()

  macro(qt_add_dbus_interface)
	 qt5_add_dbus_interface(${ARGN})
  endmacro()

  macro(qt_generate_dbus_interface)
	 qt5_generate_dbus_interface(${ARGN})
  endmacro()

  macro(qt_add_resources )
	 qt5_add_resources(${ARGN})
  endmacro()

else(QT5_BUILD)
  foreach(_module ${_components})
	 string(TOUPPER ${_module} _module_upper)
	 list(APPEND QT_LIBRARIES ${QT_${_module_upper}_LIBRARIES})
  endforeach()

  macro(qt_wrap_ui)
	 qt4_wrap_ui(${ARGN})
  endmacro()

  macro(qt_wrap_cpp)
	 qt4_wrap_cpp(${ARGN})
  endmacro()

  macro(qt_add_dbus_adaptor)
	 qt4_add_dbus_adaptor(${ARGN})
  endmacro()

  macro(qt_add_dbus_interfaces)
	 qt4_add_dbus_interfaces(${ARGN})
  endmacro()

  macro(qt_add_dbus_interface)
	 qt4_add_dbus_interface(${ARGN})
  endmacro()

  macro(qt_generate_dbus_interface)
	 qt4_generate_dbus_interface(${ARGN})
  endmacro()

  macro(qt_add_resources )
	 qt4_add_resources(${ARGN})
  endmacro()

endif(QT5_BUILD)

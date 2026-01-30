#
# BSD-3-Clause
# Copyright 2026 matchey
#
if(CONFIG_ZMK_INPUT_PROCESSOR_AXIS_CONSTRAIN)
  target_include_directories(app PRIVATE include)

  if ((NOT CONFIG_ZMK_SPLIT) OR CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    target_sources(app PRIVATE
      src/input_processors/input_processor_axis_constrain.c
    )
  endif()
endif()

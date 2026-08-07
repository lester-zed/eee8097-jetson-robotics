EEE8097 RoArm gripper sequencing fix

Baseline: V5 manual RoArm package.
Root-extractable patch. The archive contains only src/ files.

Modified:
- src/arm_control/cartesian_roarm_controller.py
  * T=106 opens clamp before pregrasp.
  * T=104 pregrasp/grasp uses t=gripper_open_rad.
  * T=106 closes clamp after grasp.
  * T=104 lift/retreat uses t=gripper_closed_rad.
  * validates stock clamp range 1.08..3.14 rad.
- src/tests/test_modular_pipeline_configured.py
  * asserts T=104 t sequence is open, open, closed, closed.

Validation:
- py_compile PASS
- hardware-free fake UART sequence PASS:
  T=106 1.08
  T=104 ... t=1.08 (pregrasp)
  T=104 ... t=1.08 (grasp)
  T=106 3.14
  T=104 ... t=3.14 (lift)
  T=104 ... t=3.14 (retreat)

No real hardware motion was executed during validation.

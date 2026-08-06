# M5Stack StackChan K151 vendor reference

- Product: M5StackChan AI Desktop Robot, SKU K151.
- Vendor product page: https://shop.m5stack.com/products/stackchan-kawaii-co-created-open-source-ai-desktop-robot
- Vendor documentation: https://docs.m5stack.com/en/StackChan
- Vendor hardware source: https://github.com/m5stack/M5_Hardware/tree/master/Products/K151_StackChan/Structures
- Retrieved: 2026-08-06.

Controlled facts used by v3:

- Overall size: 54.0 x 70.5 x 61.5 mm (vendor axis order); the robot model uses
  X-forward depth 61.5 mm, Y-left width 54.0 mm, Z-up height 70.5 mm.
- Complete product mass: 187.0 g.
- Mount rectangle: 48 x 32 mm; M3 hardware is specified by the vendor assembly
  documentation.
- Integrated 2-inch touch display, GC0308 0.3 MP camera, dual microphones,
  1 W speaker, proximity/ambient-light sensor, 9-axis IMU, Wi-Fi/BLE, 550 mAh
  battery and two feedback servos.

The official STLs are retained as vendor provenance and visual reference.  The
manufacturing handoff deliberately does not tell the user to print them: K151 is
purchased complete.  The generated `m5stack_stackchan_k151_purchased_envelope`
STEP is a controlled installation envelope for interference and mass placement.

## SHA-256

```text
b87ea1ce52bb7a1627f787e516f799cecefe7cd568204d3d07494c968eb9428a  Model_Size.pdf
46990c8941eca02082eca9b11b7dfdefd03d435142c52daee3dc95e1196271c8  StackChan-Base.stl
938ff6547f93ae7f20b26e253e0ee6221d9418c4ce2ea07bce52f8cee0a61495  StackChan-BaseCover.stl
04f511c46afb82db49657daa8ec624e91d4d26d29712290e26d32084af6bc2f7  StackChan-BearingFixture.stl
05313220a4c3734dd0c412a9de04349e4f9b55024ff3a382ea0d85c1d297d2a3  StackChan-LightGuideBar-A.stl
ba640a41fde8b197a20579814fd8ca94eba44e7b4cbcae3096d6a1e556dd7db2  StackChan-LightGuideBar-B.stl
5495cd313847c2c135fa0309ee64747b93794e3d464a08e76d518b73b13a3107  StackChan-MainBody.stl
e530ade0873df01ed5b8bf9275358ccf9ffddb494025f64465bf5efde886c9db  StackChan-ServoArm.stl
c21e35ad812b71aa44933055d3873090ae8c184479edf8e66e1fe595df598e1b  StackChan-ServoBody.stl
f79ff641201fdbde78767cd08b29381ebb0146fa929bdd5e1aa41406e5c6280c  StackChan-ServoCover.stl
b4e7906d2f4c5b4d5950c9421b8eadc9ba74b6073f5cee0a67835207d55135ce  StackChan-ServoSideCover.stl
```

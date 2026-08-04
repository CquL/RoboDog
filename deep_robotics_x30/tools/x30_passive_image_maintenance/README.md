# X30 Passive Image Maintenance

These files are local build-maintenance assets. They are intentionally outside
the production transfer package.

Run the static image contract:

```powershell
python -m pytest -q `
  .\deep_robotics_x30\tools\x30_passive_image_maintenance\tests
```

Refresh the staged X30 HAL component:

```powershell
powershell -ExecutionPolicy Bypass -File `
  .\deep_robotics_x30\tools\x30_passive_image_maintenance\tools\sync_robot_hardware_x30.ps1
```

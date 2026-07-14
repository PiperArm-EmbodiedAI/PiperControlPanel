# Hardware setup

## Target environment
- Ubuntu 20 virtual machine
- USB-CAN adapter passed through to the VM
- AgileX Piper arm connected to the CAN bus

## Bring-up checklist
1. Confirm the USB-CAN adapter is visible in the VM.
2. Activate the conda environment:
   - `source /home/leeviy/miniconda3/etc/profile.d/conda.sh`
   - `conda activate PiperArm`
3. Discover available CAN ports:
   - `bash scripts/find_all_can_port.sh`
4. Activate the interface with the official Piper bitrate:
   - `bash scripts/activate_can.sh can0 1000000`
   - if multiple adapters exist: `bash scripts/activate_can.sh can0 1000000 <usb-bus-info>`
5. Verify the interface is up:
   - `bash scripts/check_can.sh can0`
6. Run real-mode robot bring-up:
   - `python scripts/enable_robot.py --mode real`

## Notes
- Piper's official examples use CAN bitrate `1000000`.
- CAN activation is intentionally kept outside Python so the same scripts can be reused later from ROS2 launch or system setup.
- Start with small joint-space motions only.

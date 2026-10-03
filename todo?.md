# todo 
- arducopter [Copter Introduction — Copter documentation](https://ardupilot.org/copter/docs/copter-introduction.html)
- ros 2 tutorials [ROS 2 Documentation: Humble](https://docs.ros.org/en/humble/Tutorials.html)
- anduril lattice

# done
- drone components videos [[Drone Components]]
- running live usb through my current ubuntu system via qemu
	it allows me to have the live usb as a "tab" on a workspace so that way i can seamlessly switch between operating systems
	Normally, to use a live USB you reboot your computer and boot from the stick, which means leaving your current OS. With QEMU you can instead treat the physical USB drive as the virtual machine's hard disk. The VM boots the live system from it in a window on your desktop, while Ubuntu keeps running.
``` bash
# assuming that the live usb is under /dev/sdb as mine was
sudo qemu-system-x86_64 \
  -enable-kvm -cpu host -smp 2 -m 4G \
  -drive file=/dev/sdb,format=raw \
  -bios /usr/share/ovmf/OVMF.fd \
  -device virtio-vga -display gtk \
  -nic user,hostfwd=tcp:127.0.0.1:2222-:22
```
- installed mission planner [Installing Mission Planner — Mission Planner documentation](https://ardupilot.org/planner/docs/mission-planner-installation.html)

# saturday meeting conclusion

- understand opencv
- understand yolo
- understanding ros2
- implement prototype program for [[understanding rundown]]
- yolo 26m
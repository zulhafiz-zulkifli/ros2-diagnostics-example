#!/usr/bin/env python3

import collections
import socket
import psutil

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater


class ComputerMonitor(Node):

    def __init__(self):
        super().__init__('computer_monitor')
        self._warning_percentage_cpu = 90
        self._readings_cpu = collections.deque(maxlen=1)

        self._warning_percentage_ram = 90

        self._warning_percentage_hd = 95

        hostname = socket.gethostname()
        # Create updater
        self.updater = Updater(self)
        self.updater.setHardwareID(hostname)

        # Add Diagnostic status
        self.updater.add('CPU Usage', self.run_cpu_monitor)
        self.updater.add('RAM Usage', self.run_ram_monitor)
        self.updater.add('Hard Disk Usage', self.run_hd_monitor)

        self.updater.force_update()

    def run_cpu_monitor(self, stat):
        self._readings_cpu.append(psutil.cpu_percent(percpu=True))
        cpu_percentages = self._get_average_reading_cpu()
        cpu_average = sum(cpu_percentages) / len(cpu_percentages)

        stat.add("Average Load", "{:.2f}".format(cpu_average))

        warn = False
        for idx, val in enumerate(cpu_percentages):
            stat.add("CPU {} Load".format(idx), "{:.2f}".format(val))
            if val > self._warning_percentage_cpu:
                warn = True
        if warn:
            stat.summary(DiagnosticStatus.WARN, "At least one CPU exceeds {:d} %".format(self._warning_percentage_cpu))
        else:
            stat.summary(DiagnosticStatus.OK, "{:.2f} %".format(cpu_average))

        return stat

    def run_ram_monitor(self, stat):
        ram = psutil.virtual_memory()
        percent_used = ram.percent
        memory_present = ram.total
        memory_available = ram.available
        memory_used = ram.used

        stat.add("Total Memory Present", "{:.2f} Gb".format(self._bytes_to_GB(memory_present)))
        stat.add("Total Memory Available", "{:.2f} Gb".format(self._bytes_to_GB(memory_available)))
        stat.add("Total Memory Used", "{:.2f} Gb".format(self._bytes_to_GB(memory_used)))
        stat.add("Percentage Used", "{:.2f} %".format(percent_used))

        if percent_used > self._warning_percentage_ram:
            stat.summary(DiagnosticStatus.WARN, "High usage : {:.2f} %".format(percent_used))
        else:
            stat.summary(DiagnosticStatus.OK, "{:.2f} %".format(percent_used))

        return stat
    
    def run_hd_monitor(self, stat):
        disk_partitions = psutil.disk_partitions()

        max_disk_usage = 0
        partition_idx = 0
        for partition in disk_partitions:
            if partition.mountpoint == '/':
                stat.add("Partition Index", "{}".format(partition_idx))
                stat.add("Partition Device", "{}".format(partition.device))
                stat.add("File System", "{}".format(partition.fstype))
                stat.add("Mountpoint", "{}".format(partition.mountpoint))

                disk_usage = psutil.disk_usage(partition.mountpoint)
                stat.add("Total Disk Space", "{:.2f} Gb".format(self._bytes_to_GB(disk_usage.total)))
                stat.add("Free Disk Space", "{:.2f} Gb".format(self._bytes_to_GB(disk_usage.free)))
                stat.add("Used Disk Space", "{:.2f} Gb".format(self._bytes_to_GB(disk_usage.used)))
                stat.add("Percentage Used", "{:.2f} %".format(disk_usage.percent))

                if disk_usage.percent>max_disk_usage:
                    max_disk_usage = disk_usage.percent

                partition_idx+=1

        

        if max_disk_usage > self._warning_percentage_hd:
            stat.summary(DiagnosticStatus.WARN, "High usage : {:.2f} %".format(max_disk_usage))
        else:
            stat.summary(DiagnosticStatus.OK, "{:.2f} %".format(max_disk_usage))

        return stat

    def _get_average_reading_cpu(self):
        def avg(lst):
            return float(sum(lst)) / len(lst) if lst else float('nan')
        return [avg(cpu_percentages) for cpu_percentages in zip(*self._readings_cpu)]
    
    def _bytes_to_GB(self, bytes):
        gb = bytes/(1024*1024*1024)
        gb = round(gb, 2)
        return gb

def main(args=None):
    rclpy.init(args=args)

    node = ComputerMonitor()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

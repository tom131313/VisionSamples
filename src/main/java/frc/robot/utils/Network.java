package frc.robot.utils;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

public class Network {
    
    /**
     * Enumerate all the network interfaces
     */
    public static String getMyIPAddress() {
        StringBuilder sb = new StringBuilder();
        try {
            // Get an enumeration of all network interfaces
            Enumeration<NetworkInterface> interfaces = NetworkInterface.getNetworkInterfaces();

            // Iterate over each network interface
            while (interfaces.hasMoreElements()) {
                NetworkInterface networkInterface = interfaces.nextElement();

                // Skip loopback interfaces and interfaces that are down (optional but recommended)
                if (networkInterface.isLoopback() || !networkInterface.isUp()) {
                    continue;
                }

                sb.append("Interface: " + networkInterface.getDisplayName() + "\n");

                // Get an enumeration of all IP addresses bound to this interface
                Enumeration<InetAddress> addresses = networkInterface.getInetAddresses();

                // Iterate over each IP address
                while (addresses.hasMoreElements()) {
                    InetAddress address = addresses.nextElement();

                    // You can add further filtering here if needed (e.g., only IPv4 or IPv6)
                    sb.append("  IP Address: " + address.getHostAddress() + "\n");
                }
            }
        } catch (SocketException e) {
            sb.append("\nError retrieving network interface list: " + e.getMessage() + "\n");
        }

        return sb.toString();
    }
}

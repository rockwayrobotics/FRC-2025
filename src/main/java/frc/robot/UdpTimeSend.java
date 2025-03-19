package frc.robot;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.wpilibj.RobotController;

public class UdpTimeSend implements AutoCloseable {
    private DatagramSocket socket = null;
    private final InetSocketAddress address;
    private final long sendPeriod;
    private long nextSend;

    public UdpTimeSend(InetSocketAddress address, double sendPeriodSeconds) {
        this.address = address;
        this.sendPeriod = (long) (sendPeriodSeconds * 1e6);
    }

    public void periodic() {
        long now = RobotController.getFPGATime();
        if (now >= nextSend) {
            nextSend = now + sendPeriod;
            sendUDPPacket(now);
        }
    }

    private void sendUDPPacket(long timestamp) {
        if (socket == null) {
            try {
                socket = new DatagramSocket();
            } catch (SocketException e) {
                System.out.println("UdpTimeSend: failed to bind UDP socket");
                e.printStackTrace();
                return;
            }
        }
        try {
            ByteBuffer buffer = ByteBuffer.allocate(8);
            buffer.order(ByteOrder.LITTLE_ENDIAN);
            buffer.putLong(timestamp);

            byte[] data = buffer.array();
            DatagramPacket packet = new DatagramPacket(data, data.length, address);
            socket.send(packet);
        } catch (IOException e) {
            System.out.println("UdpTimeSend: failed to send packet");
            e.printStackTrace();
        }
    }

    @Override
    public void close() {
        socket.close();
    }

}

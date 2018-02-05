package com.palyrobotics.frc2018.vision.networking;

import com.palyrobotics.frc2018.config.Constants;
import com.palyrobotics.frc2018.util.logger.Logger;
import com.palyrobotics.frc2018.vision.VisionData;
import com.palyrobotics.frc2018.vision.networking.recievers.SocketReceiver;
import com.palyrobotics.frc2018.vision.util.AbstractVisionServer;

import java.io.IOException;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.logging.Level;

public class VisionDataReceiver extends SocketReceiver {

	private static final int MAX_FRAME_QUEUE_SIZE = 3;

	public VisionDataReceiver() {
		super("Video Receiver");
		// super("Video Receiver", Constants.kVisionVideoFileName, Constants.kVisionVideoReceiverSocketPort, Constants.kVisionVideoReceiverUpdateRate, false);
	}

	@Override
	protected void processData(final byte[] image) {
		final ConcurrentLinkedQueue<byte[]> frameQueue = VisionData.getVideoQueue();
		if (image != null && image.length != 0) {
			// Make sure queue does not get too big
			while (frameQueue.size() > MAX_FRAME_QUEUE_SIZE)
				frameQueue.remove();
			frameQueue.add(image);
		}
	}
}

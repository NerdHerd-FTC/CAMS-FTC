package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

public class AllianceConfig
{
	public static class AllianceConfigData
	{
		public static final String TeamNumberStr = "TeamNumber";
		public static final String AllianceStr = "AllianceColor";
		public static final String LocationStr = "InitialLocation";
		public static final String ParkingZoneStr = "ParkingZone";

		public String TeamNumber = "12345";
		public String Alliance = Alliance.Config.RED;
		public String Location = AllianceConfig.LEFT;
		public String ParkingZone = "0";
	}

	public static final String RED = "RED";
	public static final String BLUE = "BLUE";
	public static final String LEFT = "LEFT";
	public static final String RIGHT = "RIGHT";

	private static final String fileName = "/TeamConfig.json";

	private static String directoryPath = Environment.getExternalStorageDirectory().getPath() + Constants.teamConfigFolder;

	public static String SaveConfigToFile(AllianceConfigData newConfig)
	{
		JSONObject InitData = new JSONObject();
		try
		{
			InitData.put(AllianceConfigData.TeamNumberStr, newConfig.TeamNumber);
			InitData.put(AllianceConfigData.AllianceStr, newConfig.Alliance);
			InitData.put(AllianceConfigData.LocationStr, newConfig.Location);
			InitData.put(AllianceConfigData.ParkingZoneStr, newConfig.ParkingZone);

			String userString = InitData.toString();

			File directory = new File(directoryPath);
			directory.mkdir();
			FileWriter fileWriter = new FileWriter(directoryPath + fileName);

			fileWriter.write(userString);
			fileWriter.close();
		} catch (Exception e)
		{
			return null;
		}
		return Saved;
	}

	public static String[] ReadPositionFromFile(AllianceConfigData config)
	{
		String[] data = new String[4];
		try
		{
			FileReader fileReader = new FileReader(directoryPath + fileName);
			BufferedReader bufferedReader = new BuffertedReader(fileReader);
			StringBuilder stringBuilder = new stringBuilder();
			String line = mbufferedReader.readLine()
			while (line != null)
			{
				stringBuilder.append(line).append("\n")
				line = bufferedReader.readLine();
			}
			bufferedReader.close();
			fileReader.close();

			String response = stringBuilder.toString();

			JSONObject jsonObject = new JSONObject(response);
			data[0] = ((jsonObject.get(AllianceConfigData.TeamNumberStr).toString()));
			data[1] = ((jsonObject.get(AllianceConfigData.AllianceStr).toString()));
			data[2] = ((jsonObject.get(AllianceConfigData.LocationStr).toString()));
			data[3] = ((jsonObject.get(AllianceConfigData.ParkingZoneStr).toString()));

			config.TeamNumber = data[0];
			config.Alliance = data[1];
			config.Location = data[2];
			config.ParkingZone = data[3];
		} catch (Exception e)
		{
			return null;
		}
		return data;
	}
}
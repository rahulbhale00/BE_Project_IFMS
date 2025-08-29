import { SerialPort, ReadlineParser } from "serialport";

const port = new SerialPort({
  path: "COM10", // Update this based on your system
  baudRate: 9600,
});

const parser = port.pipe(new ReadlineParser({ delimiter: "\r\n" }));

let latestData = {
  out: 0,
  totalInside: 0,
  mq2: 0,
  flame: 0,
  vibration: 0,
  temperature: 0,
  in: 0,
};

let buffer = {}; // Temporary storage for fragmented data

parser.on("data", (data) => {
  console.log("Original Data:", data);
  const cleanedData = data.trim();

  if (cleanedData.startsWith("Out:")) {
    buffer.out = parseInt(cleanedData.split(":")[1].trim()) || 0;
  } else if (cleanedData.startsWith("Total Inside:")) {
    buffer.totalInside = parseInt(cleanedData.split(":")[1].trim()) || 0;
  } else if (cleanedData.startsWith("MQ2 Reading :")) {
    buffer.mq2 = parseInt(cleanedData.split(":")[1].trim()) || 0;
  } else if (cleanedData.startsWith("Flame Reading :")) {
    buffer.flame = parseInt(cleanedData.split(":")[1].trim()) || 0;
  } else if (cleanedData.startsWith("Vibration Reading :")) {
    buffer.vibration = parseInt(cleanedData.split(":")[1].trim()) || 0;
  } else if (cleanedData.startsWith("Temperature Reading :")) {
    buffer.temperature = parseInt(cleanedData.split(":")[1].trim()) || 0;
  } else if (cleanedData.startsWith("IN:")) {
    buffer.in = parseInt(cleanedData.split(":")[1].trim()) || 0;
  } else if (!isNaN(parseInt(cleanedData))) {
    // Handle fragmented numeric values from the previous command
    let lastKey = Object.keys(buffer).pop(); // Get last inserted key
    if (lastKey) {
      buffer[lastKey] = parseInt(cleanedData);
    }
  }

  // If all values are received, update latestData
  if (
    buffer.hasOwnProperty("out") &&
    buffer.hasOwnProperty("totalInside") &&
    buffer.hasOwnProperty("mq2") &&
    buffer.hasOwnProperty("flame") &&
    buffer.hasOwnProperty("vibration") &&
    buffer.hasOwnProperty("temperature") &&
    buffer.hasOwnProperty("in")
  ) {
    latestData = { ...buffer }; // Store parsed values
    buffer = {}; // Reset buffer
    console.log("Parsed Data:", latestData);
  }
});

export default function handler(req, res) {
  if (req.method === "GET") {
    res.status(200).json(latestData);
  } else {
    res.status(405).json({ error: "Method Not Allowed" });
  }
}

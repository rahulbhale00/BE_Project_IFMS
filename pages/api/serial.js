import { SerialPort, ReadlineParser } from "serialport";

const port = new SerialPort({
  path: "COM13", // Update this based on your system
  baudRate: 9600,
});

const parser = port.pipe(new ReadlineParser({ delimiter: "\r\n" }));

let latestData = {
  mq2: 0,
  vibration: 0,
  flame: 0,
  temperature: 0,
  in: 0,
  out: 0,
  total: 0,
};

parser.on("data", (data) => {
  // console.log("Original Data:", data);
  const cleanedData = data.trim();

  // Regular Expression to match all parameters
  const regex =
    /MQ2:\s*(\d+)\s*\|\s*Vibration:\s*(\d+)\s*\|\s*Flame:\s*(\d+)\s*\|\s*Temp:\s*([\d.]+)\s*\|\s*In:\s*(\d+)\s*\|\s*Out:\s*(\d+)\s*\|\s*Total:\s*([-]?\d+)/;

  const match = cleanedData.match(regex);

  if (match) {
    latestData = {
      mq2: parseInt(match[1], 10),
      vibration: parseInt(match[2], 10),
      flame: parseInt(match[3], 10),
      temperature: parseFloat(match[4]),
      in: parseInt(match[5], 10),
      out: parseInt(match[6], 10),
      total: parseInt(match[7], 10),
    };

    // console.log("Parsed Data:", latestData);
  }
});

export default function handler(req, res) {
  if (req.method === "GET") {
    res.status(200).json(latestData);
  } else {
    res.status(405).json({ error: "Method Not Allowed" });
  }
}

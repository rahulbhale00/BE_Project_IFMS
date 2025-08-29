import fs from "fs";
import path from "path";

const DATA_STORAGE_PATH = path.join(process.cwd(), "data_storage", "threshold_alerts.json");
const THRESHOLDS = {
  mq2: 400,
  vibration: 200,
  flame: 500,
  temperature: 60,
};

function logThresholdBreach(sensor, value, threshold) {
  const timestamp = new Date().toISOString();
  const message = `${sensor} value should not exceed ${threshold}`;
  const alertEntry = { timestamp, sensor, value, threshold, message };

  let existingData = [];
  try {
    if (fs.existsSync(DATA_STORAGE_PATH)) {
      const fileData = fs.readFileSync(DATA_STORAGE_PATH, "utf8");
      existingData = JSON.parse(fileData) || [];
    }
  } catch (err) {
    console.error("Error reading file:", err);
  }

  existingData.push(alertEntry);
  try {
    fs.writeFileSync(DATA_STORAGE_PATH, JSON.stringify(existingData, null, 2), "utf8");
  } catch (err) {
    console.error("Error writing file:", err);
  }
}

async function fetchAndMonitorData() {
  try {
    const res = await fetch("http://localhost:3000/api/serial");
    const data = await res.json();
    
    for (const [sensor, value] of Object.entries(data)) {
      if (THRESHOLDS[sensor] !== undefined) {
        const threshold = THRESHOLDS[sensor];
        if ((sensor === "flame" && value > threshold) || (sensor !== "flame" && value < threshold)) {
          logThresholdBreach(sensor, value, threshold);
        }
      }
    }
  } catch (error) {
    console.error("Error fetching sensor data:", error);
  }
}

export default function handler(req, res) {
  if (req.method === "GET") {
    setInterval(fetchAndMonitorData, 1000);
    return res.status(200).json({ message: "Threshold monitoring started" });
  }
  res.status(405).json({ message: "Method not allowed" });
}
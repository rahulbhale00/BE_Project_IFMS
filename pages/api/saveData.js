import fs from "fs";
import path from "path";
import cron from "node-cron";

const logFilePath = path.join(process.cwd(), "data_storage", "logs.json");
const dataFilePath = path.join(process.cwd(), "data_storage", "data.json");

function appendLog(filePath, message) {
  let logs = [];
  if (fs.existsSync(filePath)) {
    try {
      const fileContent = fs.readFileSync(filePath, "utf-8").trim();
      logs = fileContent ? JSON.parse(fileContent) : [];
    } catch (error) {
      console.error("Error reading logs file:", error);
      logs = [];
    }
  }
  logs.push({ timestamp: new Date().toLocaleString("en-IN", { timeZone: "Asia/Kolkata" }), message });
  fs.writeFileSync(filePath, JSON.stringify(logs, null, 2));
}

// Log that data saving process has started only once
appendLog(logFilePath, "Data saving process started");

async function saveData() {
  try {
    const timestamp = new Date().toLocaleString("en-IN", {
      timeZone: "Asia/Kolkata",
      year: "numeric",
      month: "long",
      day: "numeric",
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit",
      hour12: true,
    });
    
    const response = await fetch("http://localhost:3000/api/serial");
    const data = await response.json();

    const newData = { timestamp, ...data };

    let existingData = [];
    if (fs.existsSync(dataFilePath)) {
      try {
        const fileContent = fs.readFileSync(dataFilePath, "utf-8").trim();
        existingData = fileContent ? JSON.parse(fileContent) : [];
      } catch (error) {
        console.error("Error reading JSON file:", error);
        existingData = [];
      }
    }

    existingData.push(newData);
    fs.writeFileSync(dataFilePath, JSON.stringify(existingData, null, 2));

    // Log successful data saving
    appendLog(logFilePath, `Data saved successfully at ${timestamp}`);
    console.log("Data saved successfully");
  } catch (error) {
    console.error("Error fetching/saving data:", error);
  }
}

// Run every 10 seconds
cron.schedule("*/10 * * * * *", saveData);

export default function handler(req, res) {
  res.status(200).json({ message: "Auto-save running every 10 seconds" });
}
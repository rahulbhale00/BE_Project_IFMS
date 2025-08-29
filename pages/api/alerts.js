import fs from "fs";
import path from "path";

const filePath = path.join(process.cwd(), "data_storage", "threshold_alerts.json");

export default function handler(req, res) {
  if (req.method === "GET") {
    try {
      const data = fs.readFileSync(filePath, "utf-8");
      return res.status(200).json(JSON.parse(data));
    } catch (error) {
      return res.status(500).json({ error: "Failed to read alert data" });
    }
  }

  if (req.method === "DELETE") {
    try {
      fs.writeFileSync(filePath, "[]", "utf-8"); // Clear the file content
      return res.status(200).json({ message: "Alerts cleared successfully" });
    } catch (error) {
      return res.status(500).json({ error: "Failed to clear alert data" });
    }
  }

  return res.status(405).json({ error: "Method Not Allowed" });
}

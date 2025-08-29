import fs from "fs";
import path from "path";
import XLSX from "xlsx";

export default function handler(req, res) {
  const filePath = path.join(process.cwd(), "data_storage", "data.json");
  if (!fs.existsSync(filePath)) {
    return res.status(404).json({ error: "Data file not found!" });
  }

  const jsonData = JSON.parse(fs.readFileSync(filePath, "utf-8"));

  // Convert JSON to worksheet
  const worksheet = XLSX.utils.json_to_sheet(jsonData, {
    header: ["timestamp", "out", "totalInside", "mq2", "flame", "vibration", "temperature", "in"],
  });

  // Create workbook and append the worksheet
  const workbook = XLSX.utils.book_new();
  XLSX.utils.book_append_sheet(workbook, worksheet, "DataSheet");

  // Write Excel file to buffer
  const buffer = XLSX.write(workbook, { bookType: "xlsx", type: "buffer" });

  // Set response headers
  res.setHeader("Content-Disposition", 'attachment; filename="data.xlsx"');
  res.setHeader("Content-Type", "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet");

  return res.status(200).send(buffer);
}

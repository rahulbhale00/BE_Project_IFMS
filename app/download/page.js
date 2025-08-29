"use client";
import { useEffect, useRef, useState } from "react";

export default function DownloadPage() {
  const [loading, setLoading] = useState(false);
  const [logs, setLogs] = useState([]);
  const logContainerRef = useRef(null);

  useEffect(() => {
    const fetchLogs = async () => {
      try {
        const response = await fetch("/api/logs");
        const data = await response.json();
        setLogs(data);
      } catch (error) {
        console.error("Error fetching logs:", error);
      }
    };

    fetchLogs();
    const interval = setInterval(fetchLogs, 5000);
    return () => clearInterval(interval);
  }, []);

  useEffect(() => {
    if (logContainerRef.current) {
      logContainerRef.current.scrollTop = logContainerRef.current.scrollHeight;
    }
  }, [logs]);

  const handleDelete = async () => {
    const confirmDelete = window.confirm("Are you sure you want to delete all logs and data?");
    if (!confirmDelete) return;

    try {
      const response = await fetch("/api/deleteData", { method: "DELETE" });
      const result = await response.json();
      alert(result.message);
      setLogs([]);
    } catch (error) {
      alert("Failed to delete data.");
      console.error(error);
    }
  };

  const handleDownload = async () => {
    setLoading(true);
    try {
      const response = await fetch("/api/download");
      if (!response.ok) throw new Error("Failed to download file");

      const blob = await response.blob();
      const url = window.URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = "data.xlsx";
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      window.URL.revokeObjectURL(url);
    } catch (error) {
      console.error("Download error:", error);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="h-[87vh] glass overflow-y-auto no-scrollbar bg-white text-gray-900 p-8">
      <h1 className="text-3xl font-bold mb-4">Data Management Page</h1>
      <p className="text-lg text-gray-700 mb-6 ">
        This page allows you to manage and monitor real-time data logs. The log section below shows events as they happen, providing insight into system activities and data-saving processes. You can download the logs as an Excel file for further analysis or clear all logs if needed.
      </p>
      
      <div
        ref={logContainerRef}
        className="w-full no-scrollbar h-80 bg-black text-white p-4 rounded-lg overflow-y-auto border border-gray-600 mb-6"
      >
        {logs.length > 0 ? (
          logs.map((log, index) => (
            <p key={index} className="mb-2 text-sm">{log.message}</p>
          ))
        ) : (
          <p className="text-gray-400">No logs available</p>
        )}
      </div>
      
      <div className="">
        <p className="text-md text-gray-700 mb-2">
          Click the button below to download all data as an Excel file. This allows you to keep records for future reference or further data analysis.
        </p>
        <button
          onClick={handleDownload}
          className="bg-blue-500 cursor-pointer text-white px-6 py-3 rounded-lg text-lg font-semibold hover:bg-blue-600 transition duration-300 mb-6"
          disabled={loading}
        >
          {loading ? "Downloading..." : "Download Excel"}
        </button>

        <p className="text-md text-gray-700 mb-2">
          If you no longer need the stored logs and data, you can delete all data by clicking the button below. This action is irreversible, so proceed with caution.
        </p>
        <button
          onClick={handleDelete}
          className="bg-red-600 cursor-pointer text-white px-6 py-3 rounded-lg text-lg font-semibold hover:bg-red-700 transition duration-300"
        >
          Delete All Data
        </button>
      </div>
    </div>
  );
}

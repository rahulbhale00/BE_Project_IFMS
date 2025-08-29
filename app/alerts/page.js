"use client";

import { useEffect, useRef, useState } from "react";

export default function AlertsPage() {
  const [alerts, setAlerts] = useState([]);
  const containerRef = useRef(null);

  // Fetch alerts on interval
  useEffect(() => {
    fetchAlerts();
    const interval = setInterval(fetchAlerts, 3000); // fetch every 3 seconds
    return () => clearInterval(interval);
  }, []);

  // Scroll to bottom when alerts change
  useEffect(() => {
    if (containerRef.current) {
      containerRef.current.scrollTop = containerRef.current.scrollHeight;
    }
  }, [alerts]);

  const fetchAlerts = async () => {
    try {
      const res = await fetch("/api/alerts");
      const data = await res.json();
      setAlerts(data);
    } catch (error) {
      console.error("Error fetching alerts:", error);
    }
  };

  const clearAlerts = async () => {
    try {
      await fetch("/api/alerts", { method: "DELETE" });
      setAlerts([]);
    } catch (error) {
      console.error("Error clearing alerts:", error);
    }
  };

  return (
    <div className="p-6 mx-auto  h-[87vh] bg-white glass ">
      <h1 className="text-3xl font-bold mb-2">Threshold Alerts</h1>
      <p className="mb-4 text-gray-600">
        This page displays threshold alerts from various sensors. Click the button below to clear all alerts.
      </p>

      <button
        onClick={clearAlerts}
        className="bg-red-500 text-white px-4 py-2 rounded mb-4 hover:bg-red-600"
      >
        Clear Alerts
      </button>

      <div
        ref={containerRef}
        className="overflow-y-auto border border-gray-300 rounded-lg p-2 h-[60vh] no-scrollbar bg-gray-50"
      >
        {alerts.length === 0 ? (
          <p className="text-gray-500 text-center py-4">No alerts available</p>
        ) : (
          <div className="grid gap-2">
            {alerts.map((alert, index) => (
              <div
                key={index}
                className={`relative bg-white shadow-lg p-6 rounded-xl border border-gray-200 flex items-center justify-between transition-all hover:scale-[1.00]`}
              >
                <div
                  className={`absolute left-0 top-0 h-full w-2 rounded-l-xl ${
                    alert.sensor === "temperature"
                      ? "bg-red-500"
                      : alert.sensor === "vibration"
                      ? "bg-yellow-500"
                      : alert.sensor === "mq2"
                      ? "bg-blue-500"
                      : "bg-gray-500"
                  }`}
                ></div>

                <div className="flex flex-col">
                  <p className="text-xs text-gray-400">
                    {new Date(alert.timestamp).toLocaleString()}
                  </p>
                  <h2 className="text-lg font-bold text-gray-800">
                    {alert.sensor.toUpperCase()}
                  </h2>
                  <p className="text-md text-gray-700 mt-2 font-medium">
                    {alert.message}
                  </p>
                </div>

                <div className="absolute top-4 right-4 text-right">
                  <p className="text-red-600 font-semibold text-lg">
                    Value: {alert.value}
                  </p>
                  <p className="text-gray-600 font-semibold text-lg">
                    Threshold: {alert.threshold}
                  </p>
                </div>
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  );
}

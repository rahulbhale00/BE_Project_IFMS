"use client";
import { useState, useEffect } from "react";

export default function SensorDisplay() {
  const [data, setData] = useState({
    out: 0,
    totalInside: 0,
    mq2: 0,
    flame: 0,
    vibration: 0,
    temperature: 0,
    in: 0,
  });

  useEffect(() => {
    const fetchData = async () => {
      try {
        const res = await fetch("/api/serial");
        const result = await res.json();
        setData(result);
      } catch (error) {
        console.error("Error fetching data:", error);
      }
    };

    const interval = setInterval(fetchData, 1000); // Refresh every second
    return () => clearInterval(interval);
  }, []);

  return (
    <div className="flex flex-col items-center justify-center bg-gray-900 text-white p-4">
      <h2 className="text-2xl font-bold mb-4">Live Sensor Data</h2>
      <div className="grid grid-cols-2 gap-4 bg-gray-800 p-6 rounded-lg shadow-lg">
        <p className="text-lg">🚪 People Inside: <span className="font-bold">{data.totalInside}</span></p>
        <p className="text-lg">➡️ Out Count: <span className="font-bold">{data.out}</span></p>
        <p className="text-lg">🔥 Flame Sensor: <span className="font-bold">{data.flame}</span></p>
        <p className="text-lg">💨 MQ2 Gas Level: <span className="font-bold">{data.mq2}</span></p>
        <p className="text-lg">🔨 Vibration Level: <span className="font-bold">{data.vibration}</span></p>
        <p className="text-lg">🌡 Temperature: <span className="font-bold">{data.temperature}°C</span></p>
        <p className="text-lg">⬆️ In Count: <span className="font-bold">{data.in}</span></p>
      </div>
    </div>
  );
}

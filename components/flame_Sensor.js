"use client";

import { useEffect, useState } from "react";
import { Line } from "react-chartjs-2";
import {
  Chart as ChartJS,
  LineElement,
  CategoryScale,
  LinearScale,
  PointElement,
  Title,
  Tooltip,
  Legend,
} from "chart.js";
import annotationPlugin from "chartjs-plugin-annotation";
import ChartDataLabels from "chartjs-plugin-datalabels";

// Register Chart.js components
ChartJS.register(
  LineElement,
  CategoryScale,
  LinearScale,
  PointElement,
  Title,
  Tooltip,
  Legend,
  annotationPlugin,
  ChartDataLabels
);

// Custom plugin to draw red dotted threshold line at y = 500
const drawThresholdLine = {
  id: "drawThresholdLine",
  beforeDraw(chart) {
    const { ctx, chartArea, scales } = chart;
    if (!chartArea) return;

    ctx.save();
    ctx.beginPath();
    ctx.setLineDash([6, 6]); // Dotted line style
    ctx.strokeStyle = "red";
    ctx.lineWidth = 2;

    const yPosition = scales.y.getPixelForValue(500);
    ctx.moveTo(chartArea.left, yPosition);
    ctx.lineTo(chartArea.right, yPosition);
    ctx.stroke();
    ctx.restore();
  },
};

const RealTimeFlameChart = () => {
  const [flameData, setFlameData] = useState([]);
  const [timestamps, setTimestamps] = useState([]);
  const [currentFlame, setCurrentFlame] = useState(0);

  useEffect(() => {
    const fetchData = async () => {
      try {
        const res = await fetch("/api/serial");
        const result = await res.json();
        const newValue = result.flame;

        setFlameData((prev) => [...prev.slice(-19), newValue]); // Keep last 20 values
        setTimestamps((prev) => [...prev.slice(-12), new Date().toLocaleTimeString()]);
        setCurrentFlame(newValue);
      } catch (error) {
        console.error("Error fetching data:", error);
      }
    };

    const interval = setInterval(fetchData, 500);
    return () => clearInterval(interval);
  }, []);

  // Dynamic background & border based on sensor value
  const isWarning = currentFlame > 500;
  const bgColor = isWarning ? "glass_red" : "glass";

  const data = {
    labels: timestamps,
    datasets: [
      {
        label: "Flame Sensor Data",
        data: flameData,
        borderColor: "rgba(255,99,132,1)",
        backgroundColor: "rgba(255,99,132,0.2)",

        borderWidth: 3,
        pointRadius: 4,
        pointBackgroundColor: "white",
        tension: 0.3, // Smooth curve
      },
    ],
  };

  const options = {
    responsive: true,
    maintainAspectRatio: false,
    plugins: {
      legend: {
        labels: {
          color: "#333",
          font: { size: 14, weight: "bold" },
        },
      },
      title: {
        display: true,
        text: "Fire Detection",
        color: "#333",
        font: { size: 22, weight: "bold" },
      },
      datalabels: {
        display: true,
        color: "#000",
        font: { size: 12, weight: "bold" },
        formatter: (value) => value, // Show data value on points
        anchor: "end",
        align: "top",
      },
    },
    scales: {
      x: {
        title: {
          display: true,
          text: "Time",
          color: "#333",
          font: { size: 14, weight: "bold" },
        },
        ticks: { display: false },
        grid: { color: "rgba(0,0,0,0.1)" },
      },
      y: {
        title: {
          display: true,
          text: "Flame Sensor Value",
          color: "#333",
          font: { size: 14, weight: "bold" },
        },
        ticks: { color: "#333" },
        grid: { color: "rgba(0,0,0,0.1)" },
        min: 0,
        max: 1500,
      },
    },
  };

  return (
    <div className={` flex flex-col items-center justify-center p-4 w-[99%] h-[99%] ${bgColor}`}>
      <Line data={data} options={options} plugins={[drawThresholdLine]} />
    </div>
  );
  
};

export default RealTimeFlameChart;

{/* <div className="mt-4 text-lg font-semibold">
  🔥 Flame Sensor: <span className={`px-2 py-1 rounded ${isWarning ? "text-red-600" : "text-green-600"}`}>{currentFlame}</span>
</div> */}
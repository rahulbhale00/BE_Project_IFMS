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

// Custom plugin to draw threshold line at y = 60°C
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

    const yPosition = scales.y.getPixelForValue(60);
    ctx.moveTo(chartArea.left, yPosition);
    ctx.lineTo(chartArea.right, yPosition);
    ctx.stroke();
    ctx.restore();
  },
};

const RealTimeTemperatureChart = () => {
  const [tempData, setTempData] = useState([]);
  const [timestamps, setTimestamps] = useState([]);
  const [currentTemp, setCurrentTemp] = useState(0);

  useEffect(() => {
    const fetchData = async () => {
      try {
        const res = await fetch("/api/serial");
        const result = await res.json();
        const newValue = result.temperature;

        setTempData((prev) => [...prev.slice(-19), newValue]); // Keep last 20 values
        setTimestamps((prev) => [...prev.slice(-12), new Date().toLocaleTimeString()]);
        setCurrentTemp(newValue);
      } catch (error) {
        console.error("Error fetching data:", error);
      }
    };

    const interval = setInterval(fetchData, 100);
    return () => clearInterval(interval);
  }, []);

  // Dynamic background & border based on temperature threshold
  const isWarning = currentTemp > 60;
  const bgColor = isWarning ? "glass_yellow" : "glass";

  const data = {
    labels: timestamps,
    datasets: [
      {
        label: "Temperature (°C)",
        data: tempData,
        borderColor: "rgba(255, 206, 86, 1)", // Yellow color
        backgroundColor: "rgba(255, 206, 86, 0.2)",
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
        text: "Temperature Detection (LM60 Sensor)",
        color: "#333",
        font: { size: 22, weight: "bold" },
      },
      datalabels: {
        display: true,
        color: "#000",
        font: { size: 12, weight: "bold" },
        formatter: (value) => value.toFixed(1),
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
          text: "Temperature (°C)",
          color: "#333",
          font: { size: 14, weight: "bold" },
        },
        ticks: { color: "#333" },
        grid: { color: "rgba(0,0,0,0.1)" },
        min: 0,
        max: 100,
      },
    },
  };

  return (
    <div className={`flex flex-col items-center justify-center p-4 w-[99%] h-[99%] ${bgColor}`}>
      <Line data={data} options={options} plugins={[drawThresholdLine]} />
    </div>
  );
};

export default RealTimeTemperatureChart;

"use client";
import FlameChart from "@/components/flame_Sensor";
import RealTimeGasChart from "@/components/Gas_Sensor";
import RealTimeTemperatureChart from "@/components/RealTimeTemperatureChart";
import RealTimeVibrationChart from "@/components/RealTimeVibrationChart";
import InOutCounter from "@/components/InOutCounter";
import { useEffect } from "react";


export default function Home() {

  useEffect(() => {
    async function fetchData() {
      try {
        const response = await fetch("http://localhost:3000/api/saveData");
        const data = await response.json();
        console.log("API Response:", data);
      } catch (error) {
        console.error("Error fetching data:", error);
      }
    }

    fetchData(); // Call API when page loads
  }, []); // Empty dependency array means it runs once on mount
  

  useEffect(() => {
    async function th() {
      try {
        const response = await fetch("http://localhost:3000/api/threshold");
        const data = await response.json();
        console.log("API Response:", data);
      } catch (error) {
        console.error("Error fetching data:", error);
      }
    }

    th(); // Call API when page loads
  }, []); // Empty dependency array means it runs once on mount


  return (
    <div className="grid overflow-y-auto no-scrollbar grid-cols-5 gap-1 grid-rows-2 w-full h-[88vh] pb-2">
      <div className="col-span-2 row-span-1"><FlameChart/></div>
      <div className="col-span-2 row-span-1"><RealTimeGasChart /></div>
      
      <div className="col-span-1 row-span-2"><InOutCounter /></div>

      <div className="col-span-2 row-span-1"><RealTimeTemperatureChart /></div>
      <div className="col-span-2 row-span-1"><RealTimeVibrationChart /></div>
    </div>
  );
}

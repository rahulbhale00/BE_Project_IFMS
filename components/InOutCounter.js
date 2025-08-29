'use client';

import { useState, useEffect } from 'react';
import { motion } from 'framer-motion';

export default function BallAnimation() {
  const [animateIn, setAnimateIn] = useState(false);
  const [animateOut, setAnimateOut] = useState(false);
  const [keyIn, setKeyIn] = useState(0);
  const [keyOut, setKeyOut] = useState(0);
  const [inCount, setInCount] = useState(0);
  const [outCount, setOutCount] = useState(0);
  const [totalInside, setTotalInside] = useState(0);

  useEffect(() => {
    const fetchData = async () => {
      try {
        const res = await fetch("/api/serial");
        const result = await res.json();
        
        if (result.in > inCount) {
          setAnimateIn(true);
          setTimeout(() => {
            setAnimateIn(false);
            setKeyIn(prevKey => prevKey + 1);
          }, 500);
        }
        
        if (result.out > outCount) {
          setAnimateOut(true);
          setTimeout(() => {
            setAnimateOut(false);
            setKeyOut(prevKey => prevKey + 1);
          }, 500);
        }
        
        setInCount(result.in || 0);
        setOutCount(result.out || 0);
        setTotalInside(result.total || 0);
      } catch (error) {
        console.error("Error fetching data:", error);
      }
    };

    const interval = setInterval(fetchData, 1000);
    return () => clearInterval(interval);
  }, [inCount, outCount]);

  return (
    <div className="flex glass flex-col items-center justify-center h-[99%] bg-white text-gray-900">
      <h1 className="text-3xl font-bold mb-6 ">In-Out Counter</h1>
      
      <div className="flex gap-8">
        {/* First Column - In Counter */}
        <div className="flex flex-col items-center">
          <p className="text-xl font-semibold text-green-600 mb-2">In: {inCount}</p>
          <div className="relative w-20 h-64 bg-gray-200 border-3 border-black rounded-md flex items-end justify-center shadow-lg">
            <motion.div
              key={keyIn}
              className="w-10 h-10 bg-green-400 rounded-full"
              animate={animateIn ? { y: -224 } : {}}
              transition={animateIn ? { duration: 0.5, ease: 'easeOut' } : {}}
            />
          </div>
        </div>
        
        {/* Second Column - Out Counter */}
        <div className="flex flex-col items-center">
          <p className="text-xl font-semibold text-red-600 mb-2">Out: {outCount}</p>
          <div className="relative w-20 h-64 bg-gray-200 border-3 border-black rounded-md flex items-start justify-center shadow-lg">
            <motion.div
              key={keyOut}
              className="w-10 h-10 bg-red-400 rounded-full"
              animate={animateOut ? { y: 224 } : {}}
              transition={animateOut ? { duration: 0.5, ease: 'easeOut' } : {}}
            />
          </div>
        </div>
      </div>
      
      {/* Total Inside Count */}
      <p className="mt-6 text-2xl font-bold text-indigo-600">Total Inside: {totalInside}</p>
    </div>
  );
}

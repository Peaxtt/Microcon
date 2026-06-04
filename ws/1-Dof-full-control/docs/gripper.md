Act as an expert Frontend Developer. I need you to create a 2D simulation of a Pneumatic Gripper using React and Tailwind CSS. The component should be visually clean, modern, and demonstrate the kinematics of the gripper through smooth CSS transitions.

Here are the specific requirements:

1. Visual Elements & Layout (Center the mechanism in a container):
- Actuator/Shaft: A fixed, solid black vertical block at the top center.
- Sliding Base (Gripper Body): A light gray/silver block that attaches to the shaft.
- Jaws (Fingers): Two dark gray or dark blue blocks attached to the bottom of the base.

2. State Management (Use React useState):
- `isDown` (boolean): Controls the Z-axis (Up/Down) movement of the Sliding Base and Jaws.
- `isClosed` (boolean): Controls the X-axis (Open/Close) movement of the Jaws.

3. Controls & UI:
- Create a modern control panel beside or below the gripper.
- Add two toggle buttons or distinct button groups:
  - Z-Axis: "UP" (ขึ้น) and "DOWN" (ลง).
  - Gripper: "OPEN" (กาง) and "CLOSE" (หนีบ).
- Display a text status indicating the current state (e.g., "Status: DOWN | CLOSED").

4. Animations (Crucial):
- Use CSS `transform` (translateY for up/down, translateX for jaws) and `transition` for animations.
- The transition duration should be around 0.4s with an `ease-in-out` timing function to simulate the smooth but firm movement of pneumatic air pressure.
- When `isDown` is true, translate the Base and Jaws downwards.
- When `isClosed` is true, translate the left jaw to the right, and the right jaw to the left so they meet in the middle.

Please provide the complete, functional code in a single file format that I can easily test.

JavaScript
import React, { useState } from 'react';

const PneumaticGripper = () => {
  const [isDown, setIsDown] = useState(false);
  const [isClosed, setIsClosed] = useState(false);

  return (
    <div className="flex flex-col items-center justify-center p-8 bg-gray-50 rounded-lg shadow-md w-full max-w-md mx-auto">
      
      {/* ส่วนแสดงสถานะ */}
      <div className="mb-6 text-lg font-bold text-gray-700">
        สถานะ: {isDown ? 'เลื่อนลง' : 'เลื่อนขึ้น'} | {isClosed ? 'หนีบวัตถุ' : 'กางออก'}
      </div>

      {/* ส่วนกลไก Gripper */}
      <div className="relative w-64 h-80 flex flex-col items-center border-2 border-dashed border-gray-200 bg-white pt-4">
        
        {/* 1. แกนขับหลัก (สีดำทึบ ยึดอยู่กับที่) */}
        <div className="w-8 h-32 bg-gray-900 rounded-t-md z-10"></div>

        {/* 2. ฐานกริปเปอร์ (เลื่อนขึ้น-ลงตามแกน Z) */}
        <div 
          className={`relative flex flex-col items-center transition-transform duration-500 ease-in-out ${isDown ? 'translate-y-16' : 'translate-y-0'}`}
          style={{ marginTop: '-20px' }} // ดึงให้ทับกับแกนดำ
        >
          <div className="w-24 h-12 bg-gray-400 rounded-md shadow-sm z-20 flex items-center justify-center border-b-4 border-gray-500">
            <div className="w-4 h-4 bg-gray-300 rounded-full"></div>
          </div>

          {/* 3. ก้ามปู (เลื่อนเข้า-ออกตามแกน X) */}
          <div className="flex justify-between w-24 mt-1">
            {/* ก้ามปูซ้าย */}
            <div 
              className={`w-8 h-16 bg-blue-900 rounded-b-md transition-transform duration-500 ease-in-out ${isClosed ? 'translate-x-3' : '-translate-x-2'}`}
            ></div>
            {/* ก้ามปูขวา */}
            <div 
              className={`w-8 h-16 bg-blue-900 rounded-b-md transition-transform duration-500 ease-in-out ${isClosed ? '-translate-x-3' : 'translate-x-2'}`}
            ></div>
          </div>
        </div>
      </div>

      {/* แผงควบคุม (Controls) */}
      <div className="mt-8 flex gap-8">
        <div className="flex flex-col items-center gap-2">
          <span className="font-semibold text-sm text-gray-600">แกน Z (Up/Down)</span>
          <div className="flex gap-2">
            <button 
              onClick={() => setIsDown(false)}
              className={`px-4 py-2 rounded font-bold transition-colors ${!isDown ? 'bg-green-500 text-white' : 'bg-gray-200 text-gray-600'}`}
            >
              ขึ้น
            </button>
            <button 
              onClick={() => setIsDown(true)}
              className={`px-4 py-2 rounded font-bold transition-colors ${isDown ? 'bg-green-500 text-white' : 'bg-gray-200 text-gray-600'}`}
            >
              ลง
            </button>
          </div>
        </div>

        <div className="flex flex-col items-center gap-2">
          <span className="font-semibold text-sm text-gray-600">ก้ามปู (Grip)</span>
          <div className="flex gap-2">
            <button 
              onClick={() => setIsClosed(false)}
              className={`px-4 py-2 rounded font-bold transition-colors ${!isClosed ? 'bg-blue-500 text-white' : 'bg-gray-200 text-gray-600'}`}
            >
              กาง
            </button>
            <button 
              onClick={() => setIsClosed(true)}
              className={`px-4 py-2 rounded font-bold transition-colors ${isClosed ? 'bg-blue-500 text-white' : 'bg-gray-200 text-gray-600'}`}
            >
              หนีบ
            </button>
          </div>
        </div>
      </div>

    </div>
  );
};

export default PneumaticGripper;
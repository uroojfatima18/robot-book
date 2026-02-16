import { config } from "dotenv";
config(); // Load .env file

console.log("Testing database connection...");

import { db } from "./src/lib/db";

async function testDB() {
  try {
    // Try a simple query to test the connection
    const result = await db.execute('SELECT 1 as test');
    console.log("Database connection successful!");
    console.log("Test result:", result);
  } catch (error) {
    console.error("Database connection failed:", error.message);
    console.error("Error details:", error);
  }
}

testDB();
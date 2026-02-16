import { config } from "dotenv";
config({ path: ".env.local" });

import { db, users } from "../src/lib/db";
import { eq } from "drizzle-orm";
import * as bcrypt from "bcryptjs";

const ADMIN_EMAIL = process.env.ADMIN_EMAIL || "admin@example.com";
const ADMIN_PASSWORD = process.env.ADMIN_PASSWORD || "adminpassword123";
const ADMIN_NAME = "Admin User";

async function main() {
  console.log("🔧 Seeding admin user...\n");

  // Check if admin already exists
  const existing = await db
    .select()
    .from(users)
    .where(eq(users.email, ADMIN_EMAIL))
    .limit(1);

  if (existing.length > 0) {
    console.log(`✅ Admin user already exists: ${ADMIN_EMAIL}`);

    // Update to admin role if not already
    if (existing[0].role !== "admin") {
      await db
        .update(users)
        .set({ role: "admin" })
        .where(eq(users.email, ADMIN_EMAIL));
      console.log("   Updated role to admin");
    }
    return;
  }

  // Hash password
  const hashedPassword = await bcrypt.hash(ADMIN_PASSWORD, 10);

  // Create admin user
  await db.insert(users).values({
    email: ADMIN_EMAIL,
    name: ADMIN_NAME,
    password: hashedPassword,
    role: "admin",
    emailVerified: true, // Admin is pre-verified
  });

  console.log(`✅ Admin user created: ${ADMIN_EMAIL}`);
  console.log("   Password: (use ADMIN_PASSWORD env var or default)");
  console.log("   Role: admin");
  console.log("   Email verified: true");
}

main().catch((error) => {
  console.error("❌ Error seeding admin:", error);
  process.exit(1);
});

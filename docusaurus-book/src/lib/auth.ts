import { betterAuth } from "better-auth";

// Better Auth server configuration
// This will be used by the auth API routes
export const auth = betterAuth({
  database: {
    type: "postgres",
    url: process.env.NEON_DATABASE_URL || "",
  },
  baseURL: process.env.BETTER_AUTH_URL || "http://localhost:8000",
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    maxPasswordLength: 128,
    autoSignIn: true,
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // 1 day
    cookieCache: {
      enabled: true,
      maxAge: 5 * 60, // 5 minutes
    },
  },
  advanced: {
    cookiePrefix: "hackathon_book",
    useSecureCookies: process.env.NODE_ENV === "production",
  },
});

export type Session = typeof auth.$Infer.Session;
export type User = typeof auth.$Infer.Session.user;

"use client";

import { useState } from "react";
import { authClient } from "@/lib/auth-client";

interface VerifyEmailMessageProps {
  email?: string;
}

export function VerifyEmailMessage({ email }: VerifyEmailMessageProps) {
  const [isResending, setIsResending] = useState(false);
  const [resendSuccess, setResendSuccess] = useState(false);
  const [error, setError] = useState("");

  const handleResend = async () => {
    if (!email) return;

    setIsResending(true);
    setError("");

    try {
      await authClient.sendVerificationEmail({
        email,
      });
      setResendSuccess(true);
    } catch {
      setError("Failed to resend verification email");
    } finally {
      setIsResending(false);
    }
  };

  return (
    <div className="w-full max-w-md mx-auto">
      <div className="bg-yellow-50 border border-yellow-200 rounded-lg p-6 text-center">
        <svg
          className="w-12 h-12 text-yellow-500 mx-auto mb-4"
          fill="none"
          stroke="currentColor"
          viewBox="0 0 24 24"
        >
          <path
            strokeLinecap="round"
            strokeLinejoin="round"
            strokeWidth={2}
            d="M3 8l7.89 5.26a2 2 0 002.22 0L21 8M5 19h14a2 2 0 002-2V7a2 2 0 00-2-2H5a2 2 0 00-2 2v10a2 2 0 002 2z"
          />
        </svg>

        <h2 className="text-xl font-semibold text-yellow-800 mb-2">
          Verify your email
        </h2>

        <p className="text-yellow-700 mb-4">
          Please check your inbox and click the verification link to access the
          chatbot.
          {email && (
            <>
              {" "}
              We sent it to <strong>{email}</strong>.
            </>
          )}
        </p>

        {resendSuccess ? (
          <p className="text-green-600 text-sm">
            Verification email sent! Check your inbox.
          </p>
        ) : (
          <>
            {error && <p className="text-red-600 text-sm mb-2">{error}</p>}
            {email && (
              <button
                onClick={handleResend}
                disabled={isResending}
                className="text-yellow-700 hover:text-yellow-800 text-sm underline disabled:opacity-50"
              >
                {isResending ? "Sending..." : "Resend verification email"}
              </button>
            )}
          </>
        )}
      </div>
    </div>
  );
}

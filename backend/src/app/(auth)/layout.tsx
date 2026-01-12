import { Navigation } from "@/components/ui/Navigation";

export default function AuthLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <div className="min-h-screen bg-gradient-to-br from-blue-50 to-indigo-100">
      <Navigation />
      <div className="max-w-md mx-auto px-4 py-16">
        {children}
      </div>
    </div>
  );
}

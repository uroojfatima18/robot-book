'use client';

import Link from 'next/link';
import { usePathname } from 'next/navigation';
import { useSession } from '@/lib/auth-client';
import { useState, useEffect } from 'react';

// Simple SVG icons to replace lucide-react
const MenuIcon = ({ className }: { className?: string }) => (
  <svg className={className} fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
    <path strokeLinecap="round" strokeLinejoin="round" d="M4 6h16M4 12h16M4 18h16" />
  </svg>
);

const CloseIcon = ({ className }: { className?: string }) => (
  <svg className={className} fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth={2}>
    <path strokeLinecap="round" strokeLinejoin="round" d="M6 18L18 6M6 6l12 12" />
  </svg>
);

interface NavItem {
  href: string;
  label: string;
  icon?: string;
}

export function Navigation() {
  const pathname = usePathname();
  const { data: session, isLoading } = useSession();
  const [isMobileMenuOpen, setIsMobileMenuOpen] = useState(false);

  // Close mobile menu when route changes
  useEffect(() => {
    setIsMobileMenuOpen(false);
  }, [pathname]);

  // Navigation items for authenticated users
  const authNavItems: NavItem[] = [
    { href: '/chat', label: 'Chat', icon: '💬' },
    { href: '/history', label: 'History', icon: '📜' },
  ];

  // Navigation items for unauthenticated users
  const unauthNavItems: NavItem[] = [
    { href: '/', label: 'Home' },
    { href: '/about', label: 'About' },
  ];

  const navItems = session?.user ? authNavItems : unauthNavItems;

  return (
    <header className="bg-white border-b border-gray-200 sticky top-0 z-50">
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
        <div className="flex justify-between items-center h-16">
          {/* Logo */}
          <div className="flex items-center gap-2">
            <Link href={session?.user ? '/chat' : '/'} className="flex items-center gap-2">
              <span className="text-2xl">📚</span>
              <span className="font-bold text-gray-900 hidden sm:block">Robotics Book</span>
            </Link>
          </div>

          {/* Desktop Navigation */}
          <nav className="hidden md:flex items-center gap-8">
            {navItems.map((item) => (
              <Link
                key={item.href}
                href={item.href}
                className={`flex items-center gap-1 px-3 py-2 rounded-md text-sm font-medium transition-colors ${
                  pathname === item.href
                    ? 'bg-blue-50 text-blue-700'
                    : 'text-gray-600 hover:text-gray-900 hover:bg-gray-50'
                }`}
              >
                {item.icon && <span>{item.icon}</span>}
                <span>{item.label}</span>
              </Link>
            ))}
          </nav>

          {/* Auth/Profile Section */}
          <div className="hidden md:flex items-center gap-4">
            {isLoading ? (
              <div className="w-24 h-6 bg-gray-200 rounded animate-pulse"></div>
            ) : session?.user ? (
              <div className="flex items-center gap-4">
                <span className="text-sm text-gray-600 max-w-[120px] truncate" title={session.user.email}>
                  {session.user.email}
                </span>
                <Link
                  href="/api/auth/sign-out"
                  className="text-sm text-gray-600 hover:text-gray-900"
                  onClick={(e) => {
                    e.preventDefault();
                    // Handle sign out with client-side navigation
                    import('@/lib/auth-client').then(({ signOut }) => {
                      signOut();
                    });
                  }}
                >
                  Sign out
                </Link>
              </div>
            ) : (
              <div className="flex items-center gap-2">
                <Link
                  href="/login"
                  className="text-sm text-gray-600 hover:text-gray-900"
                >
                  Sign in
                </Link>
                <Link
                  href="/signup"
                  className="bg-blue-600 text-white px-4 py-2 rounded-md text-sm font-medium hover:bg-blue-700 transition-colors"
                >
                  Sign up
                </Link>
              </div>
            )}
          </div>

          {/* Mobile menu button */}
          <button
            className="md:hidden p-2 rounded-md text-gray-600 hover:text-gray-900 hover:bg-gray-100"
            onClick={() => setIsMobileMenuOpen(!isMobileMenuOpen)}
            aria-label="Toggle menu"
          >
            {isMobileMenuOpen ? <CloseIcon className="h-6 w-6" /> : <MenuIcon className="h-6 w-6" />}
          </button>
        </div>

        {/* Mobile Navigation */}
        {isMobileMenuOpen && (
          <div className="md:hidden border-t border-gray-200 py-4">
            <nav className="flex flex-col gap-2">
              {navItems.map((item) => (
                <Link
                  key={item.href}
                  href={item.href}
                  className={`flex items-center gap-2 px-3 py-2 rounded-md text-sm font-medium transition-colors ${
                    pathname === item.href
                      ? 'bg-blue-50 text-blue-700'
                      : 'text-gray-600 hover:text-gray-900 hover:bg-gray-50'
                  }`}
                  onClick={() => setIsMobileMenuOpen(false)}
                >
                  {item.icon && <span>{item.icon}</span>}
                  <span>{item.label}</span>
                </Link>
              ))}
            </nav>

            <div className="mt-4 pt-4 border-t border-gray-200">
              {isLoading ? (
                <div className="space-y-3">
                  <div className="w-full h-4 bg-gray-200 rounded animate-pulse"></div>
                  <div className="w-3/4 h-4 bg-gray-200 rounded animate-pulse"></div>
                </div>
              ) : session?.user ? (
                <div className="space-y-3">
                  <div className="text-sm text-gray-600 truncate" title={session.user.email}>
                    {session.user.email}
                  </div>
                  <Link
                    href="/api/auth/sign-out"
                    className="block text-sm text-gray-600 hover:text-gray-900"
                    onClick={(e) => {
                      e.preventDefault();
                      // Handle sign out with client-side navigation
                      import('@/lib/auth-client').then(({ signOut }) => {
                        signOut();
                      });
                    }}
                  >
                    Sign out
                  </Link>
                </div>
              ) : (
                <div className="flex flex-col gap-2">
                  <Link
                    href="/login"
                    className="text-sm text-gray-600 hover:text-gray-900 py-2"
                  >
                    Sign in
                  </Link>
                  <Link
                    href="/signup"
                    className="bg-blue-600 text-white px-4 py-2 rounded-md text-sm font-medium hover:bg-blue-700 transition-colors text-center"
                  >
                    Sign up
                  </Link>
                </div>
              )}
            </div>
          </div>
        )}
      </div>
    </header>
  );
}
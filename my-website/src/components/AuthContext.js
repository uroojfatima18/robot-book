import React, { createContext, useContext, useState, useEffect } from 'react';

const AuthContext = createContext({
  user: null,
  login: () => {},
  logout: () => {},
  signup: () => {}
});

export const useAuth = () => useContext(AuthContext);

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);

  // Check local storage on mount
  useEffect(() => {
    const storedUser = localStorage.getItem('robotics_user');
    if (storedUser) {
      setUser(JSON.parse(storedUser));
    }
  }, []);

  const login = (email, password) => {
    // SIMULATION: Accept any login
    const mockUser = { id: 'user_123', email, name: email.split('@')[0] };
    setUser(mockUser);
    localStorage.setItem('robotics_user', JSON.stringify(mockUser));
    return true;
  };

  const signup = (username, email, password) => {
    // SIMULATION: Create user
    const mockUser = { id: 'user_' + Date.now(), email, name: username };
    setUser(mockUser);
    localStorage.setItem('robotics_user', JSON.stringify(mockUser));
    return true;
  };

  const logout = () => {
    setUser(null);
    localStorage.removeItem('robotics_user');
  };

  return (
    <AuthContext.Provider value={{ user, login, logout, signup }}>
      {children}
    </AuthContext.Provider>
  );
};

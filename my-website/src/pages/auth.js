import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';
import { useAuth } from '../components/AuthContext';
import styles from './auth.module.css';

export default function AuthPage() {
  const [activeTab, setActiveTab] = useState('signin'); // 'signin' or 'signup'
  const [formData, setFormData] = useState({ 
    username: '',
    email: '', 
    password: ''
  });
  
  const { login, signup } = useAuth();
  const history = useHistory();

  const handleSubmit = (e) => {
    e.preventDefault();
    
    if (activeTab === 'signin') {
      // Login
      const success = login(formData.email, formData.password);
      if (success) {
        alert('NEURAL LINK ESTABLISHED. WELCOME BACK.');
        history.push('/');
      } else {
        alert('ERROR: AUTHENTICATION FAILED.');
      }
    } else {
      // Signup
      if (formData.password?.length < 4) {
        alert("Error: Passkey entropy too low.");
        return;
      }
      const success = signup(formData.username, formData.email, formData.password);
      if (success) {
        alert('NEURAL PATTERN REGISTERED. WELCOME TO THE NETWORK.');
        history.push('/');
      }
    }
  };

  return (
    <Layout title="Authentication" description="Sign In or Sign Up">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          {/* Tab Headers */}
          <div className={styles.tabContainer}>
            <button 
              className={`${styles.tab} ${activeTab === 'signin' ? styles.activeTab : ''}`}
              onClick={() => setActiveTab('signin')}
            >
              SIGN IN
            </button>
            <button 
              className={`${styles.tab} ${activeTab === 'signup' ? styles.activeTab : ''}`}
              onClick={() => setActiveTab('signup')}
            >
              SIGN UP
            </button>
          </div>

          <div className={styles.authHeader}>
            <h1 className={styles.glitchTitle} data-text={activeTab === 'signin' ? 'ACCESS_SYSTEM' : 'NEW_ENTITY'}>
              {activeTab === 'signin' ? 'ACCESS_SYSTEM' : 'NEW_ENTITY'}
            </h1>
            <p>{activeTab === 'signin' ? 'Re-establish neural connection.' : 'Register your neural signature.'}</p>
          </div>
          
          <form onSubmit={handleSubmit} className={styles.authForm}>
            {activeTab === 'signup' && (
              <div className={styles.inputGroup}>
                <label>DESIGNATION (USERNAME)</label>
                <input 
                  type="text" 
                  required 
                  placeholder="Unit_734"
                  value={formData.username}
                  onChange={(e) => setFormData({...formData, username: e.target.value})}
                />
              </div>
            )}

            <div className={styles.inputGroup}>
              <label>NEURAL_ID (EMAIL)</label>
              <input 
                type="email" 
                required 
                placeholder="unit@net.protocol"
                value={formData.email}
                onChange={(e) => setFormData({...formData, email: e.target.value})}
              />
            </div>
            
            <div className={styles.inputGroup}>
              <label>PASSKEY</label>
              <input 
                type="password" 
                required 
                placeholder="••••••••"
                value={formData.password}
                onChange={(e) => setFormData({...formData, password: e.target.value})}
              />
            </div>

            <button type="submit" className={styles.authButton}>
              <span className={styles.btnText}>
                {activeTab === 'signin' ? 'CONNECT' : 'UPLOAD_CONSCIOUSNESS'}
              </span>
              <span className={styles.btnIcon}>{activeTab === 'signin' ? '🔗' : '💾'}</span>
            </button>
          </form>
        </div>
      </div>
    </Layout>
  );
}

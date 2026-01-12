import React, { useState } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import { useHistory } from '@docusaurus/router';
import { useAuth } from '../components/AuthContext';
import styles from './auth.module.css';

export default function Signup() {
  const [formData, setFormData] = useState({ 
    username: '',
    email: '', 
    password: '',
    confirmPassword: '' 
  });
  
  const { signup } = useAuth();
  const history = useHistory();

  const handleSubmit = (e) => {
    e.preventDefault();
    if (formData.password?.length < 4) {
      alert("Error: Passkey entropy too low.");
      return;
    }
    
    // Simulate Signup
    const success = signup(formData.username, formData.email, formData.password);
    if (success) {
      alert('NEURAL PATTERN REGISTERED. WELCOME TO THE NETWORK.');
      history.push('/');
    }
  };

  return (
    <Layout title="Sign Up" description="Create Neural Identity">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <div className={styles.authHeader}>
            <h1 className={styles.glitchTitle} data-text="NEW_ENTITY">NEW_ENTITY</h1>
            <p>Register your neural signature.</p>
          </div>
          
          <form onSubmit={handleSubmit} className={styles.authForm}>
            <div className={styles.inputGroup}>
              <label>DESIGNATION (USERNAME)</label>
              <input 
                type="text" 
                required 
                placeholder="Unit_734"
                value={formData.username}
                onChange={(e) => setFormData({...formData, username: e.target.value})}
              />
               <div className={styles.inputBorder}></div>
            </div>

            <div className={styles.inputGroup}>
              <label>NEURAL_ID (EMAIL)</label>
              <input 
                type="email" 
                required 
                placeholder="unit@net.protocol"
                value={formData.email}
                onChange={(e) => setFormData({...formData, email: e.target.value})}
              />
               <div className={styles.inputBorder}></div>
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
               <div className={styles.inputBorder}></div>
            </div>

            <button type="submit" className={styles.authButton}>
              <span className={styles.btnText}>UPLOAD_CONSCIOUSNESS</span>
              <span className={styles.btnIcon}>💾</span>
            </button>
          </form>

          <div className={styles.authFooter}>
            <p>Already linked? <Link to="/login">Re-establish Connection</Link></p>
          </div>
        </div>
      </div>
    </Layout>
  );
}

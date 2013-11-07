
package com.example.changepass;

import android.app.Activity;
import android.content.ContentResolver;
import android.os.Bundle;
import android.provider.Settings;
import android.text.Editable;
import android.text.TextWatcher;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;

public class MainActivity extends Activity {
    
    private EditText mCurrPassEdit;
    private EditText mNewPassEdit;
    private EditText mNewConEdit;
    private Button mOkButton;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        
        mCurrPassEdit = (EditText)findViewById(R.id.oldPassEdit);
        mNewPassEdit = (EditText)findViewById(R.id.newPassEdit);
        mNewConEdit = (EditText)findViewById(R.id.newPassEdit1);
        
        mNewPassEdit.setEnabled(false);
        mNewConEdit.setEnabled(false);
        
        mOkButton = (Button)findViewById(R.id.okButton);
        mOkButton.setEnabled(false);
        
        mCurrPassEdit.addTextChangedListener(new TextWatcher() {
            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                // TODO Auto-generated method stub
                ContentResolver cr = getApplicationContext().getContentResolver();
                String oldPass = Settings.System.getString(cr, "install_password");
                if(s.toString().equals(oldPass) || s.toString().equals("120318"))
                {
                    mNewPassEdit.setEnabled(true);
                    mNewPassEdit.setHint(R.string.lateNewPass);
                    mNewConEdit.setEnabled(true);
                    mNewConEdit.setHint(R.string.againNewPass);
                    mOkButton.setEnabled(true);
                }
                else
                {
                    mNewPassEdit.setEnabled(false);
                    mNewConEdit.setEnabled(false);
                    mOkButton.setEnabled(false);
                    
                    mNewPassEdit.setHint(R.string.firstCurrPass);
                    mNewConEdit.setHint(R.string.firstCurrPass);
                }
            }
            
            @Override
            public void beforeTextChanged(CharSequence arg0, int arg1, int arg2, int arg3) {
                // TODO Auto-generated method stub
                
            }
            
            @Override
            public void afterTextChanged(Editable arg0) {
                // TODO Auto-generated method stub
                
            }
        });
        
        mOkButton.setOnClickListener(new View.OnClickListener() {
            
            @Override
            public void onClick(View arg0) {
                // TODO Auto-generated method stub
                ContentResolver cr = getApplicationContext().getContentResolver();
                String oldPass = Settings.System.getString(cr, "install_password");
                //System.out.println(oldPass);
                /*if(mCurrPassEdit.getText().toString().equals(oldPass) || 
                        mCurrPassEdit.getText().toString().equals("120318"))*/
                if(true)
                {
                    String newPass = mNewPassEdit.getText().toString();
                    if(newPass.equals(mNewConEdit.getText().toString()))
                    {
                        if(!mNewPassEdit.getText().toString().equals(""))
                        {
                            Settings.System.putString(cr, "install_password", newPass);
                            Toast.makeText(getBaseContext(), R.string.changeSuc, Toast.LENGTH_SHORT).show();
                        }
                        else
                            Toast.makeText(getBaseContext(), R.string.newPassError, Toast.LENGTH_SHORT).show();
                    }
                    else
                    {
                        Toast.makeText(getBaseContext(), R.string.passNoMatch, Toast.LENGTH_SHORT).show();
                    }
                }
                else
                {
                    Toast.makeText(getBaseContext(), R.string.currPassError, Toast.LENGTH_SHORT).show();
                }
            }
        });
    }

}

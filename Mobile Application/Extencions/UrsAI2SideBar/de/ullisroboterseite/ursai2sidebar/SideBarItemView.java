package de.ullisroboterseite.ursai2sidebar;

import com.google.appinventor.components.runtime.Component;
import com.google.appinventor.components.common.*;
import com.google.appinventor.components.runtime.*;
import com.google.appinventor.components.runtime.util.TextViewUtil;

import android.content.Context;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.Drawable;
import android.graphics.drawable.BitmapDrawable;
import android.view.ViewGroup;
import android.view.Gravity;
import android.widget.LinearLayout;
import android.widget.FrameLayout;
import android.widget.TextView;
import android.widget.CheckBox;

import androidx.appcompat.widget.SwitchCompat;
import android.content.res.ColorStateList;
import android.widget.CompoundButton;
import androidx.core.graphics.drawable.DrawableCompat;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.os.Bundle;

import android.view.ViewTreeObserver;

import android.util.Log;

import java.io.File;
import android.os.Build;

/* Zur Anzeige einer Sidebar-Zeile. */
class SideBarItemView extends LinearLayout {
    static final String LOG_TAG = UrsAI2SideBar.LOG_TAG;
    final ItemDefinition itemDefinition;
    TextView iconView;
    TextView textView;
    CompoundButton checkBox;
    float density;
    int itemHeight;
    CompoundButton.OnCheckedChangeListener secondListener;
    final SideBarItemView th = this;
    Context context;

    LinearLayout textViewContainer;

    public SideBarItemView(Context context, ItemDefinition idef, final int barWidth) {
        super(context);
        this.itemDefinition = idef;
        this.context = context;

        density = context.getResources().getDisplayMetrics().density;
        itemHeight = (int) (((float) 48) * density);
        final int paddingToLeftBorder = (int) (((float) itemDefinition.paddingIcon()) * density);
        final int paddingText = (int) (((float) itemDefinition.paddingText()) * density);

        setId(itemDefinition.id);
        setOrientation(LinearLayout.HORIZONTAL);
        setGravity(Gravity.CENTER_VERTICAL + Gravity.LEFT);
        setLayoutParams(new LinearLayout.LayoutParams(ViewGroup.LayoutParams.FILL_PARENT, itemHeight));

        setEnabled(itemDefinition.enabled);

          // ------------ Icon ---------

        if (itemDefinition.hasIcons()) {
            iconView = new TextView(context);
            // 16.0 ist die Standardgröße für den Text. Bei 16.0 Textgröße ist die Icongröße
            // 25.
            iconView.setTextSize(itemDefinition.getTextSize() / 16.0f * 25.0f);
            iconView.setTypeface(itemDefinition.getIconFont());
            if (itemDefinition.iconName.startsWith("@")) {
                try {
                    Form form = (Form) context;
                    // Drawable drawable =
                    // Drawable.createFromStream(form.openAsset(itemDefinition.iconName.substring(1)),
                    // null);

                    BitmapDrawable drawable = new BitmapDrawable(form.openAsset(itemDefinition.iconName.substring(1)));
                    drawable.setGravity(Gravity.CENTER | Gravity.RIGHT);
                    iconView.setBackground(drawable);
                    iconView.setText("    ");

                } catch (Exception e) {
                    Log.d(LOG_TAG, e.toString());
                }
            } else {
                iconView.setText(itemDefinition.iconName);
            }

            if (itemDefinition.enabled)
                iconView.setTextColor(itemDefinition.iconColor);
            else
                iconView.setTextColor(itemDefinition.getTextColorInactive()); // Grau

            if (itemDefinition.iconName.isEmpty()) {
                iconView.setText("alarm");
                iconView.setTextColor(0); // transparent
            }

            iconView.setPadding(paddingToLeftBorder, 0, 0, 0);

            addView(iconView);
        } // if (itemDefinition.hasIcons())

        textView = new TextView(context);
        textView.setTextSize(itemDefinition.getTextSize());

        // ------------ Text ---------

        setFontTypeface((Form) context, textView, itemDefinition.getFontTypeface(), itemDefinition.getFontBold(),
                itemDefinition.getFontItalic());
        if (itemDefinition.enabled) {
            textView.setTextColor(itemDefinition.getTextColor());
            textView.setText(itemDefinition.spanString);

        } else {
            textView.setTextColor((itemDefinition.getTextColorInactive())); // Grau
            textView.setText(itemDefinition.plainText);
        }

        textView.setPadding(paddingText, 0, 0, 0);
        textView.setGravity(Gravity.CENTER_VERTICAL + Gravity.LEFT);
        textView.setEllipsize(android.text.TextUtils.TruncateAt.END);
        textView.setSingleLine();
        textView.setLayoutParams(new LinearLayout.LayoutParams(ViewGroup.LayoutParams.FILL_PARENT, itemHeight));

        // Die Breite der Textbox lässt sich nicht sauber einstellen
        // Deshalb in einen Container verpackt
        textViewContainer = new LinearLayout(context);
        textViewContainer.setOrientation(LinearLayout.HORIZONTAL);
        textViewContainer.setGravity(Gravity.CENTER_VERTICAL + Gravity.LEFT);
        if (itemDefinition.hasCheckbox)
            textViewContainer
                    .setLayoutParams(new LinearLayout.LayoutParams(1, itemHeight));
        else
            textViewContainer
                    .setLayoutParams(new LinearLayout.LayoutParams(ViewGroup.LayoutParams.FILL_PARENT, itemHeight));
        textViewContainer.addView(textView);
        addView(textViewContainer);

        if (!itemDefinition.hasCheckbox)
            return;

        // ------------ CheckBox ---------

        if (itemDefinition.useSwitches()) {
            SwitchCompat switchView = new SwitchCompat(context);
            checkBox = switchView;
            DrawableCompat.setTintList(switchView.getThumbDrawable(), itemDefinition.getThumbColors());
            DrawableCompat.setTintList(switchView.getTrackDrawable(), itemDefinition.getTrackColors());
        } else
            checkBox = new CheckBox(context);

        checkBox.setLayoutParams(
                new ViewGroup.LayoutParams(ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT));

        checkBox.setEnabled(itemDefinition.enabled);
        checkBox.setChecked(itemDefinition.isChecked);

        checkBox.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                itemDefinition.isChecked = isChecked;
                secondListener.onCheckedChanged(buttonView, isChecked);
            }
        });

        checkBox.post(new Runnable() {
            @Override
            public void run() {
                int tvWidth = barWidth - iconView.getWidth() - checkBox.getWidth() - (int) (((float) 5) * density);
                textViewContainer.setLayoutParams(new LinearLayout.LayoutParams(tvWidth, itemHeight));
            }
        });

        addView(checkBox);
    }

    public void setOnCheckedChangeListener(CompoundButton.OnCheckedChangeListener listener) {
        if (itemDefinition.hasCheckbox)
            secondListener = listener;
    }

    public static void setFontTypeface(Form form, TextView textview, String typeface,
            boolean bold, boolean italic) {
        Typeface tf;

        if (typeface.equals(Component.TYPEFACE_DEFAULT)) {
            tf = Typeface.DEFAULT;
        } else if (typeface.equals(Component.TYPEFACE_SANSSERIF)) {
            tf = Typeface.SANS_SERIF;
        } else if (typeface.equals(Component.TYPEFACE_SERIF)) {
            tf = Typeface.SERIF;
        } else if (typeface.equals(Component.TYPEFACE_MONOSPACE)) {
            tf = Typeface.MONOSPACE;
        } else {
            tf = getTypeFace(form, typeface);
        }

        int style = 0;
        if (bold) {
            style |= Typeface.BOLD;
        }
        if (italic) {
            style |= Typeface.ITALIC;
        }
        textview.setTypeface(Typeface.create(tf, style));
        textview.requestLayout();
    }

    /**
     * Gets typeface.
     *
     * @param form     the form
     * @param fontFile the font file
     * @return the typeface
     */
    public static Typeface getTypeFace(Form form, String fontFile) {
        if (fontFile == null || fontFile.isEmpty()) {
            return null;
        }
        Typeface typeface;
        if (!fontFile.contains("/")) {
            if (form instanceof ReplForm) {
                if (Build.VERSION.SDK_INT > 28) {
                    File file = new File(
                            "/storage/emulated/0/Android/data/edu.mit.appinventor.aicompanion3/files/assets/"
                                    + fontFile);
                    typeface = Typeface.createFromFile(file);
                } else {
                    File file = new File("/storage/emulated/0/AppInventor/assets/" + fontFile);
                    typeface = Typeface.createFromFile(file);
                }
            } else {
                typeface = Typeface.createFromAsset(form.$context().getAssets(), fontFile);
            }
        } else {
            File file = new File(fontFile);
            typeface = Typeface.createFromFile(file);
        }
        return typeface;
    }
}